
#include <iostream>
#include <string>
#include <Windows.h>
#include <GL/glew.h>
#include <ctime>

#include <cstdint>

#include "Stopwatch.h"

#define SDL_MAIN_HANDLED
#include <SDL/SDL.h>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720


#pragma comment( lib, "winmm.lib" )

#define AIR_FLUID_TYPE 3
#define BASIC_FLUID_TYPE 5



#include "LiquidSystem.h"


/*
 *	TODO:
 *
 *		- Solids
 *		- Fundamental fluid equations
 *		- Manual forces
 *		- Fluid rendering
 *
 */




void pause( )
{
	std::string line;
	std::getline( std::cin, line, '\n' );
}

float clamp( float v, float minv, float maxv )
{
	return max( minv, min( v, maxv ) );
}


//	For benchmarking
void run_sim( LiquidSystemData * system, int numIterations, int numThreads, int numParticles )
{
	liquid_clear_system( system );

	liquid_config_set_thread_count( system, numThreads );
	liquid_activate_system( system );

	liquid_config_set_gravity( system, 0.0f );

	int actualParticles = 0;
	for( int i = 0; i < numParticles; i++ )
	{
		if( liquid_add_particle( system, 10.0f + ((WINDOW_WIDTH - 20.0f) / numParticles) * i, rand( ) % (WINDOW_HEIGHT - 20) + 10, 0.0f, 0.0f, BASIC_FLUID_TYPE ) )
			++actualParticles;
	}



	float positionsBuffer[1000];
	Stopwatch sw;
	for( int k = 0; k < numIterations; k++ )
	{
		liquid_begin_sim( system, 0.0001f );
		liquid_sync_sim( system );

		//numParticles = liquid_generate_particles_list( system, positionsBuffer, 1000 );

		/*for( int i = 0; i < 3; i++ )
		{
		std::cout << "p" << i << ": (" << positionsBuffer[i * 2] << ", " << positionsBuffer[i * 2 + 1] << ")" << std::endl;
		}*/

		//std::cout << std::endl;
	}

	float time = sw.elapsedTime( );

	std::cout << "Simulating " << actualParticles << " particles for " << numIterations << " steps on " << numThreads << " thread(s): " << time * 1000.0f << "ms";
	std::cout << " (" << floorf( numIterations / time ) << " fps)";
	std::cout << std::endl;
}

/* Rendering functions */

void renderCells( LiquidSystemData * system )
{
	glColor3f( 0.2f, 0.2f, 0.2f );
	for( int c = 0; c < LIQUID_TOTAL_CELLS; c++ )
	{
		glBegin( GL_LINE_LOOP );
		LiquidCellData * cell = system->currentState->cells + c;
		glVertex2f( cell->x, cell->y );
		glVertex2f( cell->x + cell->width, cell->y );
		glVertex2f( cell->x + cell->width, cell->y + cell->height );
		glVertex2f( cell->x, cell->y + cell->height );
		glEnd( );
	}
}

void renderSolids( LiquidSystemData * system )
{
	glColor3f( 0.5f, 0.5f, 0.5f );
	glBegin( GL_QUADS );
	for( int s = 0; s < LIQUID_MAX_SOLIDS; s++ )
	{
		//	NOTE: Shouldn't be accessing this data externally
		if( !system->solids.solidsStateData[s] )
			continue;

		float sx = system->solids.solidsPositionData[s * 4];
		float sy = system->solids.solidsPositionData[s * 4 + 1];
		float ex = system->solids.solidsPositionData[s * 4 + 2];
		float ey = system->solids.solidsPositionData[s * 4 + 3];

		glVertex2f( sx, sy );
		glVertex2f( ex, sy );
		glVertex2f( ex, ey );
		glVertex2f( sx, ey );
	}
	glEnd( );
}

void renderParticle( float x, float y, float radius )
{
	const int accuracy = 10;

	glBegin( GL_TRIANGLE_FAN );

	float progression = 0.0f;
	for( int i = 0; i <= accuracy; i++ )
	{
		glVertex2f( x + cosf( progression ) * radius, y - sinf( progression ) * radius );
		progression += M_PI * 2.0f / accuracy;
	}

	glEnd( );
}

void render( float * data, int * types, std::size_t particleCount )
{
	/*glPointSize( 2.0f );

	glBegin( GL_POINTS );
	glColor3f( 1.0f, 1.0f, 1.0f );
	for( int i = 0; i < particleCount; i++ )
	{
		switch( types[i] )
		{
			case(BASIC_FLUID_TYPE) : glColor3f( 0.5f, 0.7f, 1.0f ); break;
			case(AIR_FLUID_TYPE) : glColor3f( 1.0f, 1.0f, 1.0f ); break;
		}
		glVertex2f( data[i * 2], data[i * 2 + 1] );
	}
	glEnd( );*/

	for( int i = 0; i < particleCount; i++ )
	{
		float radius = 0.0f;
		switch( types[i] )
		{
			case( BASIC_FLUID_TYPE ):
				glColor3f( 0.5f, 0.7f, 1.0f );
				radius = 10.0f;
				break;
		}

		renderParticle( data[i * 2], data[i * 2 + 1], radius );
	}
}

int main( )
{
	srand( time( nullptr ) );

	SDL_Window * window = SDL_CreateWindow( "Water", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_OPENGL );
	SDL_GLContext context = SDL_GL_CreateContext( window );

	if( !window )
		return 0;

	SDL_GL_SetSwapInterval( 1 );

	glViewport( 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT );
	glMatrixMode( GL_PROJECTION );
	const int border = 25.0;
	gluOrtho2D( -border, WINDOW_WIDTH + border, WINDOW_HEIGHT + border, -border );

	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );

	std::uint8_t * liquidSystemMemory = new std::uint8_t[256 * 1024 * 1024];
	LiquidSystemData * system = LiquidSystemData::FromMemoryBlock( liquidSystemMemory );

	std::cout << "total cells: " << LIQUID_TOTAL_CELLS << std::endl;
	std::cout << "theoretical max particles: " << LIQUID_CELL_MAX_PARTICLES * LIQUID_TOTAL_CELLS << std::endl;

	liquid_set_particle_type( system, AIR_FLUID_TYPE, 50.0f, 5.0f, "Air" );
	liquid_set_particle_type( system, BASIC_FLUID_TYPE, 10.0f, 5.0f, "Basic Fluid" );
	int numParticles = 0, numIterations = 100;
	/*run_sim( system, numIterations, 1, numParticles );
	run_sim( system, numIterations, 2, numParticles );
	run_sim( system, numIterations, 3, numParticles );
	run_sim( system, numIterations, 4, numParticles );
	run_sim( system, numIterations, 5, numParticles );
	run_sim( system, numIterations, 6, numParticles );
	run_sim( system, numIterations, 7, numParticles );*/

	liquid_add_solid( system, 0.0f, 0.0f, 50.0f, WINDOW_HEIGHT );
	liquid_add_solid( system, WINDOW_WIDTH, 0.0f, -50.0f, WINDOW_HEIGHT );
	liquid_add_solid( system, 0.0f, 0.0f, WINDOW_WIDTH, 50.0f );
	liquid_add_solid( system, 0.0f, WINDOW_HEIGHT, WINDOW_WIDTH, -50.0f );

	liquid_clear_system( system );
	//liquid_config_set_simulation_type( system, LiquidSimulationType::Newtonian );
	liquid_config_set_simulation_type( system, LiquidSimulationType::Particle );
	liquid_config_set_gravity( system, 100.0f );
#ifdef _DEBUG
	liquid_config_set_thread_count( system, 1 );
#else
	liquid_config_set_thread_count( system, 4 );
#endif
	liquid_activate_system( system );

	liquid_add_solid( system, 200.0f, 200.0f, 200.0f, 100.0f );

	for( int i = 0; i < numParticles; i++ )
	{
		liquid_add_particle( system, 10.0f + ((WINDOW_WIDTH - 20.0f) / numParticles) * i, rand( ) % (WINDOW_HEIGHT - 20) + 10, 0.0f, 0.0f, AIR_FLUID_TYPE );
	}
	
	int bufferSize = LIQUID_TOTAL_CELLS * LIQUID_CELL_MAX_PARTICLES;
	float * positionsBuffer = new float[2 * LIQUID_TOTAL_CELLS * LIQUID_CELL_MAX_PARTICLES];
	int * typeBuffer = new int[LIQUID_TOTAL_CELLS * LIQUID_CELL_MAX_PARTICLES];

	const float step = 1.0f / 60.0f;

	DWORD startTime = timeGetTime( );
	int numFrames = 0;
	
	bool quit = false;
	bool createParticle = false;
	bool removeParticles = false;
	bool showCells = false;
	bool pause = false;
	bool singleStep = false;
	bool resetVelocity = false;
	int createTimer = 0;
	while( !quit )
	{
		SDL_Event evt;
		SDL_PumpEvents( );
		while( SDL_PollEvent( &evt ) )
		{
			if( evt.type == SDL_QUIT )
				quit = true;

			if( evt.type == SDL_MOUSEBUTTONDOWN )
			{
				if( evt.button.button == 1 )
					createParticle = true;

				if( evt.button.button == 3 )
					removeParticles = true;
			}

			if( evt.type == SDL_MOUSEBUTTONUP )
			{
				createParticle = false;
				removeParticles = false;
			}

			if( evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_g )
				showCells = !showCells;

			if( evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_p )
				pause = !pause;

			if( evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_s )
				singleStep = true;

			if( evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_d )
				resetVelocity = true;
		}


		int mx, my;
		SDL_GetMouseState( &mx, &my );

		const int numSubiterations = 1;
		for( int i = 0; i < numSubiterations && (!pause || singleStep); i++)
		{
			liquid_begin_sim( system, step / ((float)numSubiterations) );
			liquid_sync_sim( system );
		}

		if( createParticle && createTimer == 0 )
		{
			for( int i = 0; i < 1; i++ )
			{
				float rx = rand( ) / (float)RAND_MAX;
				float ry = rand( ) / (float)RAND_MAX;
				const float rvar = 0.0f;

				float newx = clamp( mx + rx * rvar - rvar / 2, 0.0f, WINDOW_WIDTH );
				float newy = clamp( my + ry * rvar - rvar / 2, 0.0f, WINDOW_HEIGHT );
				liquid_add_particle( system, newx, newy, 0.0f, 0.0f, BASIC_FLUID_TYPE );
			}

			createTimer = 30;
		}

		if( removeParticles )
		{
			float removeSize = 350.0f;
			liquid_remove_particles_rect( system, mx - removeSize * 0.5f, my - removeSize * 0.5f, removeSize, removeSize );
		}

		createTimer--;
		createTimer = max( 0, createTimer );

		if( resetVelocity )
			liquid_reset_system_energy( system );
		int numParticles = liquid_generate_particles_list( system, positionsBuffer, typeBuffer, bufferSize );
		singleStep = false;
		resetVelocity = false;

		glClear( GL_COLOR_BUFFER_BIT );
		if( showCells )
			renderCells( system );
		renderSolids( system );
		render( positionsBuffer, typeBuffer, numParticles );

		SDL_GL_SwapWindow( window );

		numFrames++;
		DWORD now = timeGetTime( );
		if( now - startTime >= 1000 )
		{
			std::cout << numFrames << "fps: " << numParticles << "\n";
			startTime = now;
			numFrames = 0;
		}
	}


	LiquidSystemData::Destroy( system );

	return 0;
}
