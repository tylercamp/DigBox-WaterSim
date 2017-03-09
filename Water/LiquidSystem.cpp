
#include "LiquidSystem.h"
#include <Windows.h>

#include <thread>

//	Different iteration strategies based on alignment for cache performance
#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_ROW_CONTIGUOUS_ALIGNMENT
# define CellLoop(x,y)								\
	for( std::size_t y = 0; y < LIQUID_NUM_CELLS; y++ )	\
		for( std::size_t x = 0; x < LIQUID_NUM_CELLS; x++ )	
#endif

#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
# define CellLoop(x,y)								\
	for( std::size_t x = 0; x < LIQUID_NUM_CELLS; x++ )	\
		for( std::size_t y = 0; y < LIQUID_NUM_CELLS; y++ )	
#endif

#define NOT_YET_IMPLEMENTED() printf("%s not yet implemented", __FUNCTION__)

#ifdef _DEBUG
# define WARN(msg) { printf("%s: %s\n", __FUNCTION__, msg); __debugbreak( ); }
//# define WARN(msg) printf("%s: %s\n", __FUNCTION__, msg)
#else
//# define WARN(msg)
# define WARN(msg) printf("%s: %s\n", __FUNCTION__, msg)
#endif

#define PARTICLE_INACTIVE false
#define PARTICLE_ACTIVE true



//	SYSTEM DEFAULTS
const float					DEFAULT_GRAVITY = 9.81f;
const float					DEFAULT_COMPRESSION_COEFFICIENT = 100.0f;
const LiquidSimulationType	DEFAULT_SIMULATION_TYPE = LiquidSimulationType::Particle;



//	Math-helpers

//	Non-0 sign
int sign( float v )
{
	return (v >= 0.0f) * 2 - 1;
}



namespace intern
{
	struct ThreadTaskData
	{
		LiquidSystemData * system;
		int startCell, endCell;

		float simTime;

		bool executeOperation;
		bool isComplete;
		bool suspend;
		bool quit;
	};

	struct LiquidSystemInternalData
	{
		/*
		 * if threadData.count == 0
		 *	then threads not yet configured, cannot start threading system
		 *
		 * if threadData.count > 0 && threads.count == 0
		 *	then threads configured, not yet started
		 *	
		 * if threadData.count > 0 && threads.count > 0
		 *	then threads configured and currently running
		 *	
		 *	
		 * ... checking the state of the threading system can be done by evaluating the above conditions
		 *
		 */

		std::vector<ThreadTaskData> threadData;
		std::vector<std::thread> threads;
	};

	inline bool assert_state_alignment( LiquidSystemState * state )
	{
		if( (std::intptr_t)state->cells % LIQUID_BYTE_ALIGNMENT )
			return false;

		for( int i = 0; i < LIQUID_NUM_CELLS * LIQUID_NUM_CELLS; i++ )
		{
			if( (std::intptr_t)state->cells[i].forceData % LIQUID_BYTE_ALIGNMENT )
				return false;

			if( (std::intptr_t)state->cells[i].velocityData % LIQUID_BYTE_ALIGNMENT )
				return false;

			if( (std::intptr_t)state->cells[i].positionData % LIQUID_BYTE_ALIGNMENT )
				return false;
		}

		return true;
	}

	inline bool assert_system_alignment( LiquidSystemData * system )
	{
		if( (std::intptr_t)&(system->a) % LIQUID_BYTE_ALIGNMENT )
			return false;

		if( (std::intptr_t)&(system->b) % LIQUID_BYTE_ALIGNMENT )
			return false;

		return assert_state_alignment( &system->a ) && assert_state_alignment( &system->b );
	}

	void get_cell_for_pos( LiquidSystemData * system, float x, float y, int * cxOut, int * cyOut )
	{
#ifdef _DEBUG
		if( !cxOut || !cyOut )
			WARN( "Both cxOut and cyOut must be specified." );

		if( x < system->stats.activeStartX || x > system->stats.activeStartX + system->stats.activeWidth ||
			y < system->stats.activeStartY || y > system->stats.activeStartY + system->stats.activeHeight )
			WARN( "Position not within system boundaries" );
#endif

		*cxOut = (int)floorf( (x - system->stats.activeStartX) / system->stats.cellWidth );
		*cyOut = (int)floorf( (y - system->stats.activeStartY) / system->stats.cellHeight );

#ifdef _DEBUG
		if( *cxOut < 0 || *cxOut >= LIQUID_NUM_CELLS ||
			*cyOut < 0 || *cyOut >= LIQUID_NUM_CELLS )
			WARN( "Incorrect cell position calculation" );
#endif
	}

	inline LiquidCellData * get_cell( LiquidSystemState * source, std::size_t x, std::size_t y )
	{
#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_ROW_CONTIGUOUS_ALIGNMENT
		return source->cells + LIQUID_NUM_CELLS * y + x;
#endif
		
#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
		return source->cells + LIQUID_NUM_CELLS * x + y;
#endif
	}

	inline void set_particle_state( LiquidCellData * cell, int particleIndex, bool state )
	{
		//	Set the 'particleIndex'th bit value to 'state'

		//	Get the specific byte containing the state
		int offset = particleIndex / 8;
#ifdef _DEBUG
		if( offset < 0 || offset >= LiquidCellData::PARTICLE_STATE_DATA_COUNT )
			WARN( "Invalid particle byte offset" );
#endif
		std::uint8_t & b = *(cell->particleStateData + offset);

		//	Wipe our specific bit
		b &= ~(1 << (particleIndex % 8));

		//	Assign the value
		b |= ((int)state) << (particleIndex % 8);
	}

	inline bool get_particle_state( LiquidCellData * cell, int particleIndex )
	{
#ifdef _DEBUG
		if( particleIndex < 0 || particleIndex > LIQUID_CELL_MAX_PARTICLES )
			WARN( "Invalid particleIndex" );
#endif

		//	Get the 'particleIndex'th bit value
		return cell->particleStateData[particleIndex / 8] & (1u << (particleIndex % 8));
	}
	
	bool solid_get_state( LiquidSystemData * system, int solidId )
	{
#ifdef _DEBUG
		if( solidId < 0 || solidId >= LIQUID_MAX_SOLIDS )
			WARN( "Invalid solidId" );
#endif

		return system->solids.solidsStateData[solidId];
	}

	void solid_set_state( LiquidSystemData * system, int solidId, bool state )
	{
#ifdef _DEBUG
		if( solidId < 0 || solidId >= LIQUID_MAX_SOLIDS )
			WARN( "Invalid solidId" );
#endif

		system->solids.solidsStateData[solidId] = state;
	}

	bool assert_cell_integrity( LiquidCellData * cell )
	{
		//	Make sure the number of bitflags assigned in particle states matches the active particle count
		std::size_t countedParticles = 0;
		for( std::size_t i = 0; i < LIQUID_CELL_MAX_PARTICLES; i++ )
		{
			countedParticles += get_particle_state( cell, i );
		}

		return cell->numActiveParticles == countedParticles;
	}

	inline void cell_set_particle_type_id( LiquidSystemData * context, LiquidCellData * cell, int particleIndex, int typeId )
	{
#ifdef _DEBUG
		//	typeId must only have its first 4 bits set (i.e. no value larger than 15) since each int in particleTypeData represents
		//		two particle types
		if( (typeId & (~0x0F)) != 0 )
			WARN( "typeId is not within valid bitrange" );

		if( context->particleDefinitions[typeId].name[0] == 0 )
			WARN( "Attempting to use undefined particle type ID" );
#endif

		std::uint8_t & b = cell->particleTypeData[particleIndex >> 1];

		//	Clear bitset for particle
		b &= 0xF0 >> ((particleIndex % 2) * 4);

		b |= typeId << ((particleIndex % 2) * 4);
	}

	inline int cell_get_particle_type_id( LiquidCellData * cell, int particleIndex )
	{
		//	Figure out which byte has our data
		std::uint8_t & b = cell->particleTypeData[particleIndex >> 1];

		return (b >> ((particleIndex % 2) * 4)) & 0x0F;
	}

	bool cell_is_active( LiquidCellData * cell )
	{
#ifdef _DEBUG
		if( !assert_cell_integrity( cell ) )
			WARN( "Cell integrity violated" );
#endif

		return cell->numActiveParticles != 0;
	}




	void set_system_cells_area( LiquidSystemData * system, float x, float y, float width, float height )
	{
		system->stats.activeStartX = x;
		system->stats.activeStartY = y;
		system->stats.activeWidth = width;
		system->stats.activeHeight = height;
		system->stats.cellWidth = width / LIQUID_NUM_CELLS + 0.001f; // Offset to correct small flooring errors when determining cell index
		system->stats.cellHeight = height / LIQUID_NUM_CELLS + 0.001f;

		CellLoop( cx, cy )
		{
			LiquidCellData & ca = *intern::get_cell( &system->a, cx, cy );
			LiquidCellData & cb = *intern::get_cell( &system->b, cx, cy );

			ca.x = x + (float)cx / LIQUID_NUM_CELLS * width;
			ca.y = y + (float)cy / LIQUID_NUM_CELLS * height;
			ca.width = width / LIQUID_NUM_CELLS;
			ca.height = height / LIQUID_NUM_CELLS;


			cb.x = x + (float)cx / LIQUID_NUM_CELLS * width;
			cb.y = y + (float)cy / LIQUID_NUM_CELLS * height;
			cb.width = width / LIQUID_NUM_CELLS;
			cb.height = height / LIQUID_NUM_CELLS;
		}
	}

	inline bool cell_intersects_aabb( LiquidCellData * cell, float x, float y, float w, float h )
	{
		return !(
			   (x > cell->x + cell->width)
			|| (x + w < cell->x)
			|| (y > cell->y + cell->height)
			|| (y + h < cell->y)
		);
	}

	inline bool point_intersects_aabb( float px, float py, float xstart, float ystart, float xend, float yend )
	{
		return	(px >= xstart && px <= xend) &&
				(py >= ystart && py <= yend);
	}

	inline bool particle_intersects_aabb( LiquidCellData * cell, int particleIndex, float x, float y, float w, float h )
	{
		float * particlePosition = cell->positionData + particleIndex * 2;
		return point_intersects_aabb( particlePosition[0], particlePosition[1], x, y, x + w, y + h );
	}

	void clear_liquid_state( LiquidSystemState * state )
	{
		CellLoop( x, y )
		{
			LiquidCellData * cell = intern::get_cell( state, x, y );

			cell->numActiveParticles = 0;
			memset( cell->particleStateData, 0, LiquidCellData::PARTICLE_STATE_DATA_COUNT );
			memset( cell->particleTypeData, 0, LiquidCellData::PARTICLE_TYPE_DATA_COUNT );

			for( int p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
			{
				cell->forceData[p * 2] = 0.0f;
				cell->velocityData[p * 2] = 0.0f;
				cell->positionData[p * 2] = 0.0f;
				cell->forceData[p * 2 + 1] = 0.0f;
				cell->velocityData[p * 2 + 1] = 0.0f;
				cell->positionData[p * 2 + 1] = 0.0f;
			}
		}
	}

	void copy_cell_data( LiquidCellData * source, LiquidCellData * target )
	{
		//	Copy particle state data
		for( int i = 0; i < LiquidCellData::PARTICLE_STATE_DATA_COUNT; i++ )
			target->particleStateData[i] = source->particleStateData[i];

		for( int i = 0; i < LiquidCellData::PARTICLE_TYPE_DATA_COUNT; i++ )
			target->particleTypeData[i] = source->particleTypeData[i];

		for( int p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
		{
			target->positionData[p * 2] = source->positionData[p * 2];
			target->positionData[p * 2 + 1] = source->positionData[p * 2 + 1];
			target->velocityData[p * 2] = source->velocityData[p * 2];
			target->velocityData[p * 2 + 1] = source->velocityData[p * 2 + 1];
		}

		target->numActiveParticles = source->numActiveParticles;
	}

	/* PROCESSING FUNCTIONS FOR PARTICLE SIMULATION */
	namespace partsys
	{
		//	Calculates the results of exerting one particle upon another, and stores the result in 'storageCell' (assumes that targetParticleIndex matches in both targetCell and storageCell)
		inline void exert_particle_specific( LiquidSystemData * context, LiquidCellData * sourceCell, LiquidCellData * targetCell, LiquidCellData * storageCell, std::size_t sourceParticleIndex, std::size_t targetParticleIndex )
		{
			const float sigma = 1.0f;

			float dx = targetCell->positionData[targetParticleIndex * 2] - sourceCell->positionData[sourceParticleIndex * 2];
			float dy = targetCell->positionData[targetParticleIndex * 2 + 1] - sourceCell->positionData[sourceParticleIndex * 2 + 1];

			//	Micro-jitter approximation - quick attempt to give value to zero-vectors that occur when particles are are similar/same position
			//	Not physically-based, equations just from what works
			float tmjx, tmjy, smjx, smjy;
			tmjx = (dy * (targetParticleIndex + 1)) * 0.0001f;
			tmjy = (dx * (targetParticleIndex + 1)) * 0.0001f;
			smjx = (dy * (sourceParticleIndex + 1)) * 0.0001f;
			smjy = (dx * (sourceParticleIndex + 1)) * 0.0001f;

			dx += tmjx - smjy;
			dy += tmjy - smjx;

			int srcType = cell_get_particle_type_id( sourceCell, sourceParticleIndex );
			int dstType = cell_get_particle_type_id( targetCell, targetParticleIndex );
			LiquidParticleDef * srcDef = context->particleDefinitions + srcType;
			LiquidParticleDef * destDef = context->particleDefinitions + dstType;

			//	Previous implementations focused on describing the forces, but the more interesting byproducts come from how the material
			//		responds to the forces; add viscocity term which determines energy loss coefficient in force calculations

			float fx = 0.0f, fy = 0.0f;

			float dist2 = dx*dx + dy*dy;
			float dist = sqrtf( dist2 ) - srcDef->radius - destDef->radius;

			float overlap = max( 0.0f, -dist / (srcDef->radius + destDef->radius) );

			if( fabs( dist ) > 0.0f )
			{
				fx = -overlap * context->properties.compressionCoefficient * dx / dist;
				fy = -overlap * context->properties.compressionCoefficient * dy / dist;
			}

			if( isinf( fx ) || isinf( fy ) || isnan( fx ) || isnan( fy ) )
				WARN( "Indeterminate simulation result" );

			storageCell->forceData[targetParticleIndex * 2] += fx;
			storageCell->forceData[targetParticleIndex * 2 + 1] += fy;
		}

		inline void exert_particle( LiquidSystemData * context, LiquidCellData * sourceCell, LiquidCellData * targetCell, std::size_t sourceParticleIndex, std::size_t targetParticleIndex )
		{
			exert_particle_specific( context, sourceCell, targetCell, targetCell, sourceParticleIndex, targetParticleIndex );
		}

#define SKIP_INACTIVE_PARTICLES

		//	Processes interactions between particles in the same cell, requires a 'before' version of the cell (used for processing comparisons)
		//		and an 'after' cell
		void process_cell_inner_interactions( LiquidSystemData * systemContext, LiquidCellData * source, LiquidCellData * target )
		{
#ifdef _DEBUG
			if( !assert_cell_integrity( source ) )
				WARN( "Cell integrity violated" );
			if( !assert_cell_integrity( target ) )
				WARN( "Cell integrity violated" );
#endif

			if( !cell_is_active( source ) )
				return;

			//	For each target particle (target of interaction)
			for( int tp = 0; tp < LIQUID_CELL_MAX_PARTICLES; tp++ )
			{
#ifdef SKIP_INACTIVE_PARTICLES
				if( get_particle_state( source, tp ) == PARTICLE_INACTIVE )
					continue;
#endif
				//	For each source particle (source of forces acting on the target particle)
				for( int sp = 0; sp < LIQUID_CELL_MAX_PARTICLES; sp++ )
				{
					if( sp == tp ) continue;
					if( get_particle_state( source, sp ) == PARTICLE_INACTIVE ) continue;

					//	FORCE ACCUMULATION
					//	Inter-fluid dynamics

					exert_particle_specific( systemContext, source, source, target, sp, tp );

					//	Manual force accumulation
				}
			}

#ifdef _DEBUG
			if( !assert_cell_integrity( target ) )
				WARN( "Cell integrity violated" );
#endif
		}

		//	Interactions between arbitrary cell 'source' and a target cell 'target'
		void process_cell_interactions( LiquidSystemData * systemContext, LiquidCellData * source, LiquidCellData * target )
		{
#ifdef _DEBUG
			if( !assert_cell_integrity( source ) )
				WARN( "Cell integrity violated" );
			if( !assert_cell_integrity( target ) )
				WARN( "Cell integrity violated" );
#endif

			//	For each target particle (target of interaction)
			for( int tp = 0; tp < LIQUID_CELL_MAX_PARTICLES; tp++ )
			{
#ifdef SKIP_INACTIVE_PARTICLES
				if( get_particle_state( target, tp ) == PARTICLE_INACTIVE )
					continue;
#endif
				//	For each source particle (source of forces acting on the target particle)
				for( int sp = 0; sp < LIQUID_CELL_MAX_PARTICLES; sp++ )
				{
					if( get_particle_state( source, sp ) == PARTICLE_INACTIVE ) continue;

					//	FORCE ACCUMULATION
					//	Inter-fluid dynamics

					exert_particle( systemContext, source, target, sp, tp );

					//	Manual force accumulation
				}
			}

#ifdef _DEBUG
			if( !assert_cell_integrity( target ) )
				WARN( "Cell integrity violated" );
#endif
		}

		void process_cell_physics( LiquidSystemData * context, LiquidCellData * source, float dt, float gravity )
		{
#ifdef _DEBUG
			if( !assert_cell_integrity( source ) )
				WARN( "Cell integrity violated" );
#endif

			for( std::size_t i = 0; i < LIQUID_CELL_MAX_PARTICLES; i++ )
			{
				if( !intern::get_particle_state( source, i ) )
					continue;

				LiquidParticleDef * particleType = context->particleDefinitions + cell_get_particle_type_id( source, i );

				float & px = source->positionData[i * 2];
				float & py = source->positionData[i * 2 + 1];
				float & vx = source->velocityData[i * 2];
				float & vy = source->velocityData[i * 2 + 1];
				vx += source->forceData[i * 2] * dt / particleType->mass;
				vy += (source->forceData[i * 2 + 1] + gravity * particleType->mass) * dt / particleType->mass;

				source->forceData[i * 2] = 0.0f;
				source->forceData[i * 2 + 1] = 0.0f;
				px += source->velocityData[i * 2] * dt;
				py += source->velocityData[i * 2 + 1] * dt;

				px = (px > 0) * px;
				py = (py > 0) * py;
				px = (px > SIM_WIDTH) * SIM_WIDTH + (px < SIM_WIDTH) * px;
				py = (py > SIM_HEIGHT) * SIM_HEIGHT + (py < SIM_HEIGHT) * py;

				/*const float border = 50.0f;
				const float avoidanceForce = 1000.0f;
				if( px < border )
				source->forceData[i * 2] = avoidanceForce * particleType->mass * (border - px) / border;
				if( px > SIM_WIDTH - border )
				source->forceData[i * 2] = -avoidanceForce * particleType->mass * (border - (SIM_WIDTH - px)) / border;
				if( py < border )
				source->forceData[i * 2 + 1] = avoidanceForce * particleType->mass * (border - py) / border;
				if( py > SIM_HEIGHT - border )
				source->forceData[i * 2 + 1] = -avoidanceForce * particleType->mass * (border - (SIM_HEIGHT - py)) / border;*/

				if( px == 0.0f || px == SIM_WIDTH ) source->velocityData[i * 2] = 0.0f;
				if( py == 0.0f || py == SIM_HEIGHT ) source->velocityData[i * 2 + 1] = 0.0f;
			}

#ifdef _DEBUG
			if( !assert_cell_integrity( source ) )
				WARN( "Cell integrity violated" );
#endif
		}

		inline void process_cell_solid_interactions( LiquidCellData * cell, float startx, float starty, float endx, float endy, float dt )
		{
			for( std::size_t p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
			{
				if( !particle_intersects_aabb( cell, p, startx, starty, endx - startx, endy - starty ) )
					continue;

				float & px = cell->positionData[p * 2];
				float & py = cell->positionData[p * 2 + 1];
				float & vx = cell->velocityData[p * 2];
				float & vy = cell->velocityData[p * 2 + 1];

				px -= vx * dt;
				py -= vy * dt;

				if( px > endx || px < startx )
					vx = 0.0f;
				if( py > endy || py < starty )
					vy = 0.0f;

				/*
				//	determine which velocity components to invert
				if( px > endx || px < startx )
				vx *= -0.3f;
				if( py > endy || py < starty )
				vy *= -0.3f;
				*/
			}
		}

		void process_cell_collisions( LiquidSystemData * context, LiquidCellData * cell, float simTime )
		{
			for( std::size_t s = 0; s < LIQUID_MAX_SOLIDS; s++ )
			{
				if( !solid_get_state( context, s ) )
					continue;

				//	solid start/end x/y
				float & sx = context->solids.solidsPositionData[s * 4],
					&sy = context->solids.solidsPositionData[s * 4 + 1],
					&ex = context->solids.solidsPositionData[s * 4 + 2],
					&ey = context->solids.solidsPositionData[s * 4 + 3];

				if( !cell_is_active( cell ) || !cell_intersects_aabb( cell, sx, sy, ex - sx, ey - sy ) )
					continue;

				process_cell_solid_interactions( cell, sx, sy, ex, ey, simTime );
			}
		}

		void thread_process_cells( LiquidSystemData * context, LiquidSystemState * source, LiquidSystemState * target, int start, int end, float simTime )
		{
			LiquidCellData *sourceCell, *targetCell;
			sourceCell = source->cells + start;
			targetCell = target->cells + start;
			for( int i = start; i < end; i++, sourceCell++, targetCell++ )
			{
#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
				int cy = i % LIQUID_NUM_CELLS;
				int cx = (i - cy) / LIQUID_NUM_CELLS;
#else
				int cx = i % LIQUID_NUM_CELLS;
				int cy = (i - cx) / LIQUID_NUM_CELLS;
#endif

				copy_cell_data( sourceCell, targetCell );

#ifdef SKIP_INACTIVE_PARTICLES
				if( !cell_is_active( sourceCell ) )
					continue;
#endif

				process_cell_inner_interactions( context, sourceCell, targetCell );

#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
				for( int x = cx - LIQUID_CELL_ADJACENCY_AWARENESS; x <= cx + LIQUID_CELL_ADJACENCY_AWARENESS; x++ )
				{
					if( x < 0 || x >= LIQUID_NUM_CELLS )
						continue;

					for( int y = cy - LIQUID_CELL_ADJACENCY_AWARENESS; y <= cy + LIQUID_CELL_ADJACENCY_AWARENESS; y++ )
					{
						if( y < 0 || y >= LIQUID_NUM_CELLS )
							continue;
#else
				for( int y = cy - LIQUID_CELL_ADJACENCY_AWARENESS; y <= cy + LIQUID_CELL_ADJACENCY_AWARENESS; y++ )
				{
					if( y < 0 || y >= LIQUID_NUM_CELLS )
						continue;

					for( int x = cx - LIQUID_CELL_ADJACENCY_AWARENESS; x <= cx + LIQUID_CELL_ADJACENCY_AWARENESS; x++ )
					{
						if( x < 0 || x >= LIQUID_NUM_CELLS )
							continue;
#endif

						if( x == cx && y == cy )
							continue;

						LiquidCellData * cell = get_cell( source, x, y );
#ifdef SKIP_INACTIVE_PARTICLES
						if( !cell_is_active( cell ) )
							continue;
#endif

						//	TODO: Should take info regarding source particles and target particles, but should also specify storage cell for parallel-correctness
						process_cell_interactions( context, cell, targetCell );
					}
				}

				//process_cell_solid_interactions( sourceCell,  )
				process_cell_collisions( context, targetCell, simTime );
				process_cell_physics( context, targetCell, simTime, context->properties.gravity );
			}
		}
	}

	/* PROCESSING FUNCTIONS FOR NEWTONIAN SIMULATION */
	namespace newtsys
	{
		void process_cell_physics( LiquidSystemData * system, LiquidCellData * targetCell, float dt, float gravity )
		{
			for( int i = 0; i < LIQUID_CELL_MAX_PARTICLES; i++ )
			{
				if( !get_particle_state( targetCell, i ) )
					continue;


			}
		}

		void thread_process_cells( LiquidSystemData * context, LiquidSystemState * source, LiquidSystemState * target, int start, int end, float simTime )
		{
			LiquidCellData *sourceCell, *targetCell;
			sourceCell = source->cells + start;
			targetCell = target->cells + start;
			for( int i = start; i < end; i++, sourceCell++, targetCell++ )
			{
#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
				int cy = i % LIQUID_NUM_CELLS;
				int cx = (i - cy) / LIQUID_NUM_CELLS;
#else
				int cx = i % LIQUID_NUM_CELLS;
				int cy = (i - cx) / LIQUID_NUM_CELLS;
#endif

				copy_cell_data( sourceCell, targetCell );

#ifdef SKIP_INACTIVE_PARTICLES
				if( !cell_is_active( sourceCell ) )
					continue;
#endif

				//	Process intersections within the cell
				//process_cell_inner_interactions( context, sourceCell, targetCell );

#if LIQUID_CELL_ALIGNMENT == LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT
				for( int x = cx - LIQUID_CELL_ADJACENCY_AWARENESS; x <= cx + LIQUID_CELL_ADJACENCY_AWARENESS; x++ )
				{
					if( x < 0 || x >= LIQUID_NUM_CELLS )
						continue;

					for( int y = cy - LIQUID_CELL_ADJACENCY_AWARENESS; y <= cy + LIQUID_CELL_ADJACENCY_AWARENESS; y++ )
					{
						if( y < 0 || y >= LIQUID_NUM_CELLS )
							continue;
#else
				for( int y = cy - LIQUID_CELL_ADJACENCY_AWARENESS; y <= cy + LIQUID_CELL_ADJACENCY_AWARENESS; y++ )
				{
					if( y < 0 || y >= LIQUID_NUM_CELLS )
						continue;

					for( int x = cx - LIQUID_CELL_ADJACENCY_AWARENESS; x <= cx + LIQUID_CELL_ADJACENCY_AWARENESS; x++ )
					{
						if( x < 0 || x >= LIQUID_NUM_CELLS )
							continue;
#endif

						if( x == cx && y == cy )
							continue;

						LiquidCellData * cell = get_cell( source, x, y );
#ifdef SKIP_INACTIVE_PARTICLES
						if( !cell_is_active( cell ) )
							continue;
#endif

						//	Process interactions between current cell and adjacent cells
						//process_cell_interactions( context, cell, targetCell );
					}
				}



				//	Process interactions between the cell and solids

				//process_cell_solid_interactions( sourceCell,  )
				//process_cell_collisions( context, targetCell, simTime );
				process_cell_physics( context, targetCell, simTime, context->properties.gravity );
			}
		}
	}

	void cell_add_particle( LiquidSystemData * context, LiquidCellData * cell, float px, float py, float vx, float vy, int particleTypeIndex )
	{
#ifdef _DEBUG
		/*if( LIQUID_CELL_MAX_PARTICLES - cell->numActiveParticles <= LIQUID_CELL_MAX_PARTICLES * 0.2 )
			WARN( "Approaching max particle count" );*/

		if( !intern::assert_cell_integrity( cell ) )
			WARN( "Cell integrity violated" );
#endif

		if( cell->numActiveParticles == LIQUID_CELL_MAX_PARTICLES )
		{
			//WARN( "Cell reached particle limit, new particle ignored" );
			return;
		}

		std::size_t i = 0;
		for( i; i < LIQUID_CELL_MAX_PARTICLES; i++ )
		{
			if( get_particle_state( cell, i ) )
				continue;

			cell->positionData[i * 2] = px;
			cell->positionData[i * 2 + 1] = py;

			cell->velocityData[i * 2] = vx;
			cell->velocityData[i * 2 + 1] = vy;

			set_particle_state( cell, i, true );
			cell->numActiveParticles += 1;

			break;
		}

#ifdef _DEBUG
		if( !get_particle_state( cell, i ) )
		{
			WARN( "Unable to set particle state to active" );
		}
#endif

		cell_set_particle_type_id( context, cell, i, particleTypeIndex );
	}

	void cell_remove_particle( LiquidCellData * cell, int particleIndex )
	{
		if( !get_particle_state( cell, particleIndex ) )
		{
			WARN( "Attempted to remove a nonexistant particle" );
			return;
		}

		set_particle_state( cell, particleIndex, false );
		cell->numActiveParticles -= 1;

#ifdef _DEBUG
		if( get_particle_state( cell, particleIndex ) )
			WARN( "Failed to set particle state to inactive" );
#endif
	}

	void cell_transfer_particle( LiquidSystemData * context, LiquidCellData * source, LiquidCellData * dest, int sourceParticleIndex )
	{
#ifdef _DEBUG
		if( !assert_cell_integrity( source ) )
			WARN( "Cell integrity violated" );
		if( !assert_cell_integrity( dest ) )
			WARN( "Cell integrity violated" );
#endif

		cell_add_particle(
			context,
			dest,
			source->positionData[sourceParticleIndex * 2], source->positionData[sourceParticleIndex * 2 + 1],
			source->velocityData[sourceParticleIndex * 2], source->velocityData[sourceParticleIndex * 2 + 1],
			cell_get_particle_type_id( source, sourceParticleIndex )
		);

#ifdef _DEBUG
		if( !assert_cell_integrity( source ) )
			WARN( "Cell integrity violated" );
		if( !assert_cell_integrity( dest ) )
			WARN( "Cell integrity violated" );
#endif

		cell_remove_particle( source, sourceParticleIndex );

#ifdef _DEBUG
		if( !assert_cell_integrity( source ) )
			WARN( "Cell integrity violated" );
#endif
	}

	//	Operation over the entire state structure (cannot be ran in separate segments, but can be ran in parallel)
	void process_particle_cell_transfers( LiquidSystemData * context, LiquidSystemState * state )
	{
		LiquidCellData * cell = state->cells;
		CellLoop( x, y )
		{
#ifdef _DEBUG
			if( !assert_cell_integrity( cell ) )
				WARN( "Cell integrity violated" );
#endif

			for( int p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
			{
				//	Might be able to avoid branching and do transfers even if the particle is inactive, unsure of the branch vs. cache pull overhead here
				if( !intern::get_particle_state( cell, p ) )
					continue;

				//	TODO: Possibly optimize, lots of branching

				float px = cell->positionData[p * 2];
				float py = cell->positionData[p * 2 + 1];

				int expectedCx, expectedCy;
				get_cell_for_pos( context, px, py, &expectedCx, &expectedCy );

				if( expectedCx != x || expectedCy != y )
					cell_transfer_particle( context, cell, get_cell( state, expectedCx, expectedCy ), p );
			}

#ifdef _DEBUG
			if( !assert_cell_integrity( cell ) )
				WARN( "Cell integrity violated" );
#endif

			++cell;
		}
	}


	void thread_main( void * p )
	{
		ThreadTaskData * taskData = (ThreadTaskData *)p;
		LiquidSystemData * system = taskData->system;
		LiquidSimulationType simType = system->properties.simulationType;
		while( !taskData->quit )
		{
			int sleepLength = (int)taskData->suspend;
			Sleep( sleepLength ); // Sleep(0) if not suspended (attempt immediate-reentry), Sleep(1) if suspended (allow minimal sleep time)
			if( !taskData->executeOperation )
				continue;

			taskData->executeOperation = false;
			switch( simType )
			{
				case( LiquidSimulationType::Particle ):
					partsys::thread_process_cells( taskData->system, taskData->system->previousState, taskData->system->currentState, taskData->startCell, taskData->endCell, taskData->simTime );
					break;

				case( LiquidSimulationType::Newtonian ):
					newtsys::thread_process_cells( taskData->system, taskData->system->previousState, taskData->system->currentState, taskData->startCell, taskData->endCell, taskData->simTime );
					break;

#ifdef _DEBUG
				default:
					WARN( "Unknown LiquidSystemData simulation type" );
					break;
#endif
			}
			taskData->isComplete = true;
		}
	}
}






LiquidSystemData * LiquidSystemData::FromMemoryBlock( std::uint8_t * memoryBlock )
{
//#if _DEBUG
	if( (std::intptr_t)memoryBlock % LIQUID_BYTE_ALIGNMENT != 0 )
	{
		fprintf( stderr, "LiquidSystemData::FromMemoryBlock - Incorrect byte alignment for provided memory block" );
		memoryBlock = BYTE_ALIGN( memoryBlock, LIQUID_BYTE_ALIGNMENT );
	}
//#endif
	LiquidSystemData * result = (LiquidSystemData *)memoryBlock;
	intern::clear_liquid_state( &(result->a) );
	intern::clear_liquid_state( &(result->b) );
	result->isStable = false;

	result->currentState = &result->a;
	result->previousState = &result->b;

	result->internal = new intern::LiquidSystemInternalData( );

	result->properties.gravity					= DEFAULT_GRAVITY;
	result->properties.compressionCoefficient	= DEFAULT_COMPRESSION_COEFFICIENT;
	result->properties.simulationType			= DEFAULT_SIMULATION_TYPE;

	intern::set_system_cells_area( result, 0.0f, 0.0f, SIM_WIDTH, SIM_HEIGHT );

	for( std::size_t i = 0; i < LIQUID_MAX_PARTICLE_TYPES; i++ )
	{
		//	If name is null, particle type ID is undefined
		result->particleDefinitions[i].name[0] = 0;
	}

	for( std::size_t i = 0; i < LIQUID_MAX_SOLIDS; i++ )
	{
		result->solids.solidsStateData[i] = false;
	}

	return result;
}

void LiquidSystemData::Destroy( LiquidSystemData * system )
{
	intern::LiquidSystemInternalData * internalData = (intern::LiquidSystemInternalData *)system->internal;
	if( internalData->threads.size( ) > 0 )
	{
		for( int i = 0; i < internalData->threads.size( ); i++ )
		{
			internalData->threadData[i].quit = true;
			internalData->threads[i].join( );
		}
	}

	delete internalData;
}


void liquid_clear_system( LiquidSystemData * system )
{
#if _DEBUG
	if( !intern::assert_system_alignment( system ) )
		printf( "Warning: LiquidSystemData not aligned to %i-byte boundary", LIQUID_BYTE_ALIGNMENT );
#endif

	//	Shut down threads
	intern::LiquidSystemInternalData * internal = (intern::LiquidSystemInternalData *)system->internal;
	for( std::size_t i = 0; i < internal->threads.size( ); i++ )
	{
		internal->threadData[i].quit = true;
		internal->threads[i].join( );
	}

	internal->threads.clear( );

	intern::clear_liquid_state( &system->a );
	intern::clear_liquid_state( &system->b );

#ifdef _DEBUG
	for( std::size_t c = 0; c < LIQUID_NUM_CELLS; c++ )
	{
		if( !intern::assert_cell_integrity( system->a.cells + c ) )
			WARN( "Cell integrity violated" );

		if( !intern::assert_cell_integrity( system->b.cells + c ) )
			WARN( "Cell integrity violated" );
	}
#endif
}

void liquid_begin_sim( LiquidSystemData * system, float deltaTime )
{
	auto internalData = (intern::LiquidSystemInternalData *)system->internal;

#ifdef _DEBUG
	if( internalData->threadData.size( ) == 0 )
		WARN( "Need to specify number of threads with liquid_config_set_thread_count at least once before calling" );
#endif

	liquid_activate_system( system );

	auto prevCurrentState = system->currentState;
	auto prevPrevState = system->previousState;

	//	Swap buffers (after processing, 'currentState' will hold new info, 'previousState' holds reference data to be processed)
	system->currentState = prevPrevState;
	system->previousState = prevCurrentState;

	for( std::size_t i = 0; i < internalData->threads.size( ); i++ )
	{
		internalData->threadData[i].isComplete = false;
		internalData->threadData[i].simTime = deltaTime;
		internalData->threadData[i].executeOperation = true;
	}
}

void liquid_sync_sim( LiquidSystemData * system )
{
	auto internalData = (intern::LiquidSystemInternalData *)system->internal;

	for( std::size_t i = 0; i < internalData->threadData.size( ); i++ )
	{
		while( !internalData->threadData[i].isComplete )
		{
			Sleep( 0 );
		}
	}

	intern::process_particle_cell_transfers( system, system->currentState );
}

void liquid_suspend_system( LiquidSystemData * system )
{
	auto internalData = (intern::LiquidSystemInternalData *)system->internal;

	for( auto & threadData : internalData->threadData )
	{
		threadData.suspend = true;
	}
}

void liquid_activate_system( LiquidSystemData * system )
{
	auto internalData = (intern::LiquidSystemInternalData *)system->internal;

#ifdef _DEBUG
	if( internalData->threadData.size( ) == 0 )
		WARN( "Need to specify number of threads with liquid_config_set_thread_count at least once before calling" );
#endif

	if( internalData->threads.size( ) == 0 )
	{
		for( std::size_t i = 0; i < internalData->threadData.size( ); i++ )
		{
			internalData->threads.push_back( std::thread( intern::thread_main, internalData->threadData.data( ) + i ) );
		}
	}

	for( std::size_t i = 0; i < internalData->threads.size( ); i++ )
		internalData->threadData[i].suspend = false;
}

void liquid_config_set_thread_count( LiquidSystemData * system, std::size_t numThreads )
{
	//	TODO: Debug check system stability

	intern::LiquidSystemInternalData * internalData = (intern::LiquidSystemInternalData *)system->internal;
	if( internalData->threads.size( ) > 0 )
	{
		for( int i = 0; i < internalData->threads.size( ); i++ )
		{
			internalData->threadData[i].quit = true;
			internalData->threads[i].join( );
		}
	}

	internalData->threadData.clear( );
	internalData->threads.clear( );

	float cellsPerThread = LIQUID_TOTAL_CELLS / (float)numThreads;
	float currentCell = 0.0f;
	for( std::size_t i = 0; i < numThreads; i++, currentCell += cellsPerThread )
	{
		//	TODO: Not entirely correct, very last cell might be getting skipped for processing
		int startCell = (int)floorf( currentCell );
		int endCell = (int)floor( currentCell + cellsPerThread );

		intern::ThreadTaskData currentThreadData;
		currentThreadData.startCell = startCell;
		currentThreadData.endCell = endCell;
		currentThreadData.quit = false;
		currentThreadData.suspend = false;
		currentThreadData.executeOperation = false;
		currentThreadData.system = system;

		internalData->threadData.push_back( currentThreadData );
	}

#ifdef _DEBUG
	//	Make sure there's no overlap between cells
	for( std::size_t i = 0; i < numThreads; i++ )
	{
		for( std::size_t j = 0; j < numThreads; j++ )
		{
			if( i == j ) continue;

			if( (internalData->threadData[i].startCell > internalData->threadData[j].startCell) != (internalData->threadData[i].endCell > internalData->threadData[j].endCell) )
				WARN( "Thread-cell assignment overlap" );
		}
	}

	//	Make sure every cell was assigned to some thread
	for( std::size_t c = 0; c < LIQUID_TOTAL_CELLS; c++ )
	{
		bool isAssigned = false;
		for( std::size_t t = 0; t < numThreads; t++ )
		{
			if( internalData->threadData[t].startCell <= c && internalData->threadData[t].endCell >= c )
			{
				isAssigned = true;
				break;
			}
		}

		if( !isAssigned )
			WARN( "Cell not assigned for thread processing" );
	}
#endif
}

void liquid_config_set_thread_count_auto( LiquidSystemData * system )
{
	NOT_YET_IMPLEMENTED( );
}

void liquid_config_set_gravity( LiquidSystemData * system, float gravityStrength )
{
	system->properties.gravity = gravityStrength;
}

void liquid_config_set_simulation_type( LiquidSystemData * system, LiquidSimulationType simType )
{
#ifdef _DEBUG
	intern::LiquidSystemInternalData * internal = (intern::LiquidSystemInternalData *)system->internal;
	if( internal->threads.size( ) > 0 )
	{
		WARN( "Must call liquid_clear_system before changing the simulation type of a re-used system" );
		return;
	}
#endif

	system->properties.simulationType = simType;
}

std::size_t liquid_generate_particles_list( LiquidSystemData * system, float * targetPositionsBuffer, int * targetTypesBuffer, std::size_t maxParticles )
{
	std::size_t count = 0;

	//	NOTE: bufferLength is the size of the buffer, count is the number of particles written into the buffer thus far

	for( std::size_t c = 0; c < LIQUID_TOTAL_CELLS && count < maxParticles; c++ )
	{
		LiquidCellData * cell = system->currentState->cells + c;
#ifdef _DEBUG
		if( !intern::assert_cell_integrity( cell ) )
			WARN( "Cell integrity violated" );
#endif

		for( std::size_t p = 0; p < LIQUID_CELL_MAX_PARTICLES && count < maxParticles; p++ )
		{
			if( !intern::get_particle_state( cell, p ) )
				continue;

			targetPositionsBuffer[count * 2] = cell->positionData[p * 2];
			targetPositionsBuffer[count * 2 + 1] = cell->positionData[p * 2 + 1];
			targetTypesBuffer[count] = intern::cell_get_particle_type_id( cell, p );

			++count;
		}

#ifdef _DEBUG
		if( !intern::assert_cell_integrity( cell ) )
			WARN( "Cell integrity violated" );
#endif
	}

	return count;
}

void liquid_set_particle_type( LiquidSystemData * system, int newTypeId, float particleRadius, float particleMass, const char * particleTypeName )
{
	LiquidParticleDef & def = system->particleDefinitions[newTypeId];
	def.radius = particleRadius;
	def.mass = particleMass;

	int nameLength = strlen( particleTypeName );
	if( nameLength >= LiquidParticleDef::MAX_NAME_LENGTH )
		WARN( "Particle name is too long" );
	if( nameLength == 0 )
		WARN( "Name must be non-0 length" );

	//	Some extra care to avoid buffer overflows (truncate name worst-case)
	nameLength = max( nameLength, LiquidParticleDef::MAX_NAME_LENGTH - 1 );
	memcpy( def.name, particleTypeName, nameLength );
	//	Remember to null-terminate
	def.name[nameLength] = 0;
}

int liquid_add_solid( LiquidSystemData * system, float x, float y, float width, float height )
{
	return liquid_add_solid_specific( system, x, y, x + width, y + height );
}

int liquid_add_solid_specific( LiquidSystemData * system, float xstart, float ystart, float xend, float yend )
{
	float xs, ys, xe, ye;
	if( xstart > xend )
	{
		xs = xend;
		xe = xstart;
	}
	else
	{
		xs = xstart;
		xe = xend;
	}

	if( ystart > yend )
	{
		ys = yend;
		ye = ystart;
	}
	else
	{
		ys = ystart;
		ye = yend;
	}

	std::size_t result = -1;
	for( std::size_t s = 0; s < LIQUID_MAX_SOLIDS; s++ )
	{
		if( intern::solid_get_state( system, s ) )
			continue;

		intern::solid_set_state( system, s, true );
		system->solids.solidsPositionData[s * 4 + 0] = xs;
		system->solids.solidsPositionData[s * 4 + 1] = ys;
		system->solids.solidsPositionData[s * 4 + 2] = xe;
		system->solids.solidsPositionData[s * 4 + 3] = ye;

		result = s;

		break;
	}

	return result;
}

void liquid_remove_solid( LiquidSystemData * system, int solidId )
{
#ifdef _DEBUG
	if( solidId < 0 || solidId >= LIQUID_MAX_SOLIDS )
		WARN( "Invalid solidId" );

	if( !intern::solid_get_state( system, solidId ) )
		WARN( "Attempting to remove a solid that is already deactivated" );
#endif

	intern::solid_set_state( system, solidId, false );
}

void liquid_query_avg_force( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, float32 * fxOut, float32 * fyOut )
{
#ifdef _DEBUG
	if( !fxOut || !fyOut )
		WARN( "fxOut and fyOut must be non-null" );
#endif

	float & fx = *fxOut;
	float & fy = *fyOut;

	fx = 0.0f;
	fy = 0.0f;

	//	TODO: Could be optimized (we can transform world coordinates to grid coordinates and get the exact cell(s)
	//		containing the rect)
	LiquidCellData * currentCell = system->currentState->cells;
	for( std::size_t c = 0; c < LIQUID_TOTAL_CELLS; c++, currentCell++ )
	{
		if( !intern::cell_intersects_aabb( currentCell, x, y, width, height ) )
			continue;

		for( std::size_t p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
		{
			//	TODO: Could optimize to avoid branching (* bool)
			if( !intern::get_particle_state( currentCell, p ) )
				continue;

			if( !intern::particle_intersects_aabb( currentCell, p, x, y, width, height ) )
				continue;

			fx += currentCell->forceData[p * 2];
			fy += currentCell->forceData[p * 2 + 1];
		}
	}
}

void liquid_query_avg_velocity( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, float32 * vxOut, float32 * vyOut )
{
#ifdef _DEBUG
	if( !vxOut || !vyOut )
		WARN( "fxOut and fyOut must be non-null" );
#endif

	float & vx = *vxOut;
	float & vy = *vyOut;

	vx = 0.0f;
	vy = 0.0f;

	//	TODO: Could be optimized (we can transform world coordinates to grid coordinates and get the exact cell(s)
	//		containing the rect)
	LiquidCellData * currentCell = system->currentState->cells;
	for( std::size_t c = 0; c < LIQUID_TOTAL_CELLS; c++, currentCell++ )
	{
		if( !intern::cell_intersects_aabb( currentCell, x, y, width, height ) )
			continue;

		for( std::size_t p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
		{
			//	TODO: Could optimize to avoid branching (* bool)
			if( !intern::get_particle_state( currentCell, p ) )
				continue;

			if( !intern::particle_intersects_aabb( currentCell, p, x, y, width, height ) )
				continue;

			vx += currentCell->velocityData[p * 2];
			vy += currentCell->velocityData[p * 2 + 1];
		}
	}
}

void liquid_reset_system_energy( LiquidSystemData * system )
{
	LiquidCellData * cell = system->currentState->cells;
	for( int c = 0; c < LIQUID_TOTAL_CELLS; c++, cell++ )
	{
		for( int p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
		{
			cell->velocityData[p * 2] = 0.0f;
			cell->velocityData[p * 2 + 1] = 0.0f;
		}
	}
}

bool liquid_add_particle( LiquidSystemData * system, float32 x, float32 y, float32 vx, float32 vy, int particleTypeIndex )
{
	int cx, cy;
	intern::get_cell_for_pos( system, x, y, &cx, &cy );

	LiquidCellData * targetCell = intern::get_cell( system->currentState, cx, cy );
	if( targetCell->numActiveParticles == LIQUID_CELL_MAX_PARTICLES )
		return false;

	//	Check that the particle isn't being spawned inside of an existing solid
	for( std::size_t s = 0; s < LIQUID_MAX_SOLIDS; s++ )
	{
		if( !intern::solid_get_state( system, s ) )
			continue;

		float xs, ys, xe, ye;
		xs = system->solids.solidsPositionData[s * 4 + 0];
		ys = system->solids.solidsPositionData[s * 4 + 1];
		xe = system->solids.solidsPositionData[s * 4 + 2];
		ye = system->solids.solidsPositionData[s * 4 + 3];

		//	If the particle intersects with the solid, not a valid particle spawn position
		if( intern::point_intersects_aabb( x, y, xs, ys, xe, ye ) )
			return false;
	}

	intern::cell_add_particle( system, targetCell, x, y, vx, vy, particleTypeIndex );

	return true;
}

void liquid_remove_particles_rect( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, LiquidParticleQueryData * resultOut )
{
	CellLoop( cx, cy )
	{
		LiquidCellData * cell = intern::get_cell( system->currentState, cx, cy );
		if( !intern::cell_intersects_aabb( cell, x, y, width, height ) )
			continue;

		for( int p = 0; p < LIQUID_CELL_MAX_PARTICLES; p++ )
		{
			if( !intern::get_particle_state( cell, p ) )
				continue;

			if( intern::particle_intersects_aabb( cell, p, x, y, width, height ) )
				intern::cell_remove_particle( cell, p );
		}
	}

	if( resultOut )
		NOT_YET_IMPLEMENTED( );
}


