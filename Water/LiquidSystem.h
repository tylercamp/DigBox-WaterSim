
#ifndef _LIQUID_SYSTEM_H_
#define _LIQUID_SYSTEM_H_

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>

#define kb(x) (1024*(x))
#define mb(x) (1024*kb(x))
#define gb(x) (1024*mb(x))


//	Not perfect, but good enough
//	Returns an aligned pointer to a buffer starting at the given address (will waste some header bytes)
#define BYTE_ALIGN(address, alignment) ((address) + (alignment) - ((std::intptr_t)(address) % (alignment)))

//	Assuming the data starts at a byte-aligned address, determines the number of elements required to end the data on a byte aligned address
//	(Again, not perfect, but good enough)
#define BYTE_ALIGN_DATA(dataSize, elements, alignment) ((elements) + ((alignment) - (((elements) * (dataSize)) % (alignment))) / dataSize)
#define BYTE_ALIGN_FLOAT_DATA(elements, alignment) BYTE_ALIGN_DATA(sizeof(float), elements, alignment)

//	Cell grid is NUMxNUM
#define LIQUID_NUM_CELLS (50)
#define LIQUID_TOTAL_CELLS (LIQUID_NUM_CELLS * LIQUID_NUM_CELLS)

//	SSE requires 16-byte alignment for operands
#define LIQUID_BYTE_ALIGNMENT 16
#ifdef _DEBUG
# define LIQUID_CELL_MAX_PARTICLES 30
#else
# define LIQUID_CELL_MAX_PARTICLES 30
#endif

//	Determines whether cells are stored as X by Y or Y by X (determines access equation + iteration strategy)
#define LIQUID_CELL_ROW_CONTIGUOUS_ALIGNMENT 1
#define LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT 2
//	With thin cell rows it's more likely that particles will transfer vertically between cells (TODO: confirm that this is the case in practice)
//	(Current benchmarks indicate 20% performance increase by having the appropriate alignment configuration)
#define LIQUID_CELL_ALIGNMENT LIQUID_CELL_COLUMN_CONTIGUOUS_ALIGNMENT

//	NOTE: While this value can be decreased, the max number of types is based on how LiquidCellData::particleTypeData uses 4-bit integer groups
//		to index into LiquidSystemData::particleDefinitions.
#define LIQUID_MAX_PARTICLE_TYPES 16

//	How many adjacent cells to check when comparing for particle collisions (i.e. at 2, collision checks are done for cells +/- 2 away on x/y for current cell
#define LIQUID_CELL_ADJACENCY_AWARENESS 2

//	Max number of solids allowed within a single system (subsequent solids are ignored)
#define LIQUID_MAX_SOLIDS 200

#define SIM_WIDTH 1280
#define SIM_HEIGHT 720



typedef float float32;



struct LiquidParticleDef
{
	float32 mass;

	// should determine a meaningful name for this property ("repulsion force"? "effective area RMS"?
	//	need to determine what calculations will be done on this property)
	float32 radius; 

	static const int MAX_NAME_LENGTH = 24;
	char name[MAX_NAME_LENGTH];
};

//	Not used directly for simulation - used for queries against the simulation and transfer of particles between cells
struct LiquidParticleData
{
	float32 x, y;
	float32 vx, vy;

	LiquidParticleDef * def;
};

//	Data regarding a solid entity within the liquid system
struct LiquidSolidData
{
	static const std::size_t SOLID_POSITION_DATA_COUNT = LIQUID_MAX_SOLIDS * 4;
	static const std::size_t SOLID_STATE_DATA_COUNT = LIQUID_MAX_SOLIDS;

	float32 solidsPositionData[SOLID_POSITION_DATA_COUNT];
	bool solidsStateData[SOLID_STATE_DATA_COUNT];
};

__declspec(align(16))
struct LiquidCellData
{
	//	2 * MAX_PARTICLES since 2 components per vector
	static const std::size_t ALIGNED_PARTICLE_VECTOR_DATA_COUNT = BYTE_ALIGN_FLOAT_DATA( 2 * LIQUID_CELL_MAX_PARTICLES, LIQUID_BYTE_ALIGNMENT );
	static const std::size_t PARTICLE_STATE_DATA_COUNT = LIQUID_CELL_MAX_PARTICLES / 8 + 1;
	static const std::size_t PARTICLE_TYPE_DATA_COUNT = LIQUID_CELL_MAX_PARTICLES / 2 + 1;

	//	SSE ALIGNED MEMBERS
	float positionData[ALIGNED_PARTICLE_VECTOR_DATA_COUNT];
	float velocityData[ALIGNED_PARTICLE_VECTOR_DATA_COUNT];
	float forceData[ALIGNED_PARTICLE_VECTOR_DATA_COUNT];

	//	NON-SSE MEMBERS

	//	bit-field for active/deactive states (must go AFTER the aligned data structures)
	std::uint8_t particleStateData[PARTICLE_STATE_DATA_COUNT];
	std::uint8_t particleTypeData[PARTICLE_TYPE_DATA_COUNT];
	std::size_t numActiveParticles;

	float x, y, width, height;

	void * threadDataQueues;
};

struct LiquidSystemState
{
	LiquidCellData cells[LIQUID_TOTAL_CELLS];
};

enum struct LiquidSimulationType
{
	//	Particles are points with forces simulated between them based on their physical properties. Forces between particles are practically 0 until their radii intersect. Basic
	//		collision detection against solids negates velocity on the axis of collision.
	Particle,

	//	Particles are treated as spheres centered around a point. Basic physics calculations are done to solve intersections/simulations of spherical collision.
	Newtonian
};

struct LiquidSystemData
{
	static LiquidSystemData * FromMemoryBlock( std::uint8_t * memoryBlock );
	static void Destroy( LiquidSystemData * system );

	//	Two buffer copies for consistent state processing during particle interaction processing
	LiquidSystemState a;
	LiquidSystemState b;

	LiquidSystemState *currentState, *previousState;

	LiquidParticleDef particleDefinitions[LIQUID_MAX_PARTICLE_TYPES];

	LiquidSolidData solids;

	//	Compile-time constant stats
	struct
	{
		float cellWidth, cellHeight;
		float activeStartX, activeStartY;
		float activeWidth, activeHeight;
	} stats;

	//	Runtime-configurable properties
	struct
	{
		float gravity;
		float compressionCoefficient;
		LiquidSimulationType simulationType;
	} properties;

	//	Whether or not the system is currently being processed
	bool isStable;

	void * internal;
};

struct LiquidParticleQueryData
{
	std::vector<LiquidParticleData> data;
};




/* for testing */
void liquid_test( LiquidSystemData * system );



//	Resets the state of the entire system. Simulation must be stable before clearing.
void liquid_clear_system( LiquidSystemData * system );

//	Begins simulation processing for a single frame, simulating deltaTime seconds.
void liquid_begin_sim( LiquidSystemData * system, float deltaTime );

//	Blocks the calling thread until simulation is complete. (Waits for isStable == true)
void liquid_sync_sim( LiquidSystemData * system );

//	Suspends simulation resources until the next call to liquid_begin_sim
void liquid_suspend_system( LiquidSystemData * system );

//	Forces acquisition of resources necessary for processing (normally done by the first call to liquid_begin_sim)
void liquid_activate_system( LiquidSystemData * system );

//	Sets the thread count for simulation. Simulation must be stable before changing config. 
void liquid_config_set_thread_count( LiquidSystemData * system, std::size_t numThreads );

//	Automatically determines an appropriate number of threads based on the system configuration at runtime
void liquid_config_set_thread_count_auto( LiquidSystemData * system );

//	Sets the gravity strength for the simulation. Simulation must be stable before changing config.
void liquid_config_set_gravity( LiquidSystemData * system, float gravityStrength );

//	Changes the global compression coefficient used in calculations. Default is 200.0f. Increasing will increase the intensity
//		of the compression/decompression forces.
void liquid_config_set_compression_coefficient( LiquidSystemData * system, float coefficient );

//	Sets the processing style of simulation (see LiquidSimulationType). If this is called after the system has been activated,
//		no effect occurs - simulation type changes will take effect after a call to liquid_clear_system(...).
void liquid_config_set_simulation_type( LiquidSystemData * system, LiquidSimulationType simulationType );

//	Intended for debug purposes - Sets the velocity of all particles to 0.
void liquid_reset_system_energy( LiquidSystemData * system );



//	Fills 'targetPositionBuffer' with the positions of particles within the system, 'targetTypesBuffer' with the type IDs of each particle, and the number of particles written
//		to the buffer is returned.
std::size_t liquid_generate_particles_list( LiquidSystemData * system, float * targetPositionBuffer, int * targetTypesBuffer, std::size_t maxParticles );



//	Registers a new particle type for 'newTypeId' with the properties specified. Will fail if the type is already in use.
void liquid_set_particle_type( LiquidSystemData * system, int newTypeId, float particleRadius, float particleMass, const char * particleTypeName );



//	Generates a new solid and returns the ID of that solid. IDs are NOT guaranteed to be unique/reproducible between additions/removals, ID values should NOT be saved/loaded to/from disk
int liquid_add_solid( LiquidSystemData * system, float x, float y, float width, float height );

//	Generates a new solid with the specified bounding points and returns the ID of that solid. IDs are NOT guaranteed to be unique/reproducible between additions/removals.
int liquid_add_solid_specific( LiquidSystemData * system, float xstart, float ystart, float xend, float yend );

//	Removes the solid with the specified ID from the system.
void liquid_remove_solid( LiquidSystemData * system, int solidId );



//	Gets average force acting upon particles in an area. Simulation must be stable before querying.
void liquid_query_avg_force( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, float32 * fxOut, float32 * fyOut );

//	Gets average velocity of particles in an area. Simulation must be stable before querying.
void liquid_query_avg_velocity( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, float32 * vxOut, float32 * vyOut );



//	Adds a particle to the system. Simulation must be stable before modifying.
bool liquid_add_particle( LiquidSystemData * system, float32 x, float32 y, float32 vx, float32 vy, int particleTypeIndex );

//	Removes all particles within the area and stores the properties of all removed particles in resultOut. Simulation must be stable before modifying.
void liquid_remove_particles_rect( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height, LiquidParticleQueryData * resultOut = nullptr );



//	Applies a force to all particles within the rect. Simulation must be stable before applying.
void liquid_apply_force_rect( LiquidSystemData * system, float32 x, float32 y, float32 width, float32 height );

//	Applies a force to all particles within the circle. Simulation must be stable before applying.
void liquid_apply_force_circle( LiquidSystemData * system, float32 x, float32 y, float32 radius );



#endif