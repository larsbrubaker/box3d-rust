// Scripted Geometric Mover run; one trace line per step (raw float bits).
#include "gm_sample.h"

static uint32_t Bits( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

struct WorldHash
{
	uint64_t h;
	int count;
};

static void HashU32( WorldHash* wh, uint32_t u )
{
	for ( int i = 0; i < 4; ++i )
	{
		wh->h ^= ( u >> ( 8 * i ) ) & 0xff;
		wh->h *= 0x00000100000001b3ull;
	}
}

// Every shape in the broad-phase, in tree query order: hash its body transform.
static bool g_dump = false;
static bool HashShape( b3ShapeId shapeId, void* context )
{
	WorldHash* wh = (WorldHash*)context;
	b3WorldTransform xf = b3Body_GetTransform( b3Shape_GetBody( shapeId ) );
	if ( g_dump )
		printf( "  shape %d body %d %08x %08x %08x %08x %08x %08x %08x\n", shapeId.index1, b3Shape_GetBody( shapeId ).index1,
				Bits( (float)xf.p.x ), Bits( (float)xf.p.y ), Bits( (float)xf.p.z ), Bits( xf.q.v.x ), Bits( xf.q.v.y ),
				Bits( xf.q.v.z ), Bits( xf.q.s ) );
	HashU32( wh, Bits( (float)xf.p.x ) );
	HashU32( wh, Bits( (float)xf.p.y ) );
	HashU32( wh, Bits( (float)xf.p.z ) );
	HashU32( wh, Bits( xf.q.v.x ) );
	HashU32( wh, Bits( xf.q.v.y ) );
	HashU32( wh, Bits( xf.q.v.z ) );
	HashU32( wh, Bits( xf.q.s ) );
	wh->count += 1;
	return true;
}

int main( int argc, char** argv )
{
	int stepCount = argc > 1 ? atoi( argv[1] ) : 900;
	GeometricMoverSample* sample = new GeometricMoverSample();
	sample->m_camera->m_thirdPerson = true;

	for ( int step = 0; step < stepCount; ++step )
	{
		Keys keys = {};
		float yawDeg = 120.0f;
		if ( 30 <= step && step < 200 )
			keys.w = true;
		if ( 200 <= step && step < 210 )
			keys.space = true;
		if ( 220 <= step && step < 320 )
		{
			yawDeg = 200.0f;
			keys.w = true;
			keys.shift = true;
		}
		if ( 330 <= step && step < 400 )
		{
			yawDeg = 200.0f;
			keys.s = true;
			keys.ctrl = true;
		}
		if ( 400 <= step && step < 600 )
		{
			yawDeg = 300.0f;
			keys.w = true;
			keys.d = true;
		}
		if ( 450 <= step && step < 455 )
			keys.space = true;
		if ( 600 <= step )
		{
			yawDeg = 30.0f;
			keys.w = true;
		}

		sample->m_keys = keys;
		sample->m_camera->m_yaw = yawDeg * DEG_TO_RAD;
		sample->m_camera->m_pitch = 30.0f * DEG_TO_RAD;
		sample->m_camera->UpdateTransform();
		if ( step == 500 )
			sample->m_mover.m_clipVelocity = false;
		if ( step == 700 )
			sample->m_mover.m_clipVelocity = true;

		sample->Step();

		const GeometricMover& m = sample->m_mover;
		printf( "%d %08x %08x %08x %08x %08x %08x %08x g%d p%d", step, Bits( (float)m.m_position.x ),
				Bits( (float)m.m_position.y ), Bits( (float)m.m_position.z ), Bits( m.m_velocity.x ), Bits( m.m_velocity.y ),
				Bits( m.m_velocity.z ), Bits( m.m_pogoVelocity ), (int)m.m_onGround, m.m_planeCount );
		if ( sample->m_boxImpact.valid )
			printf( " box %08x", Bits( sample->m_boxImpact.speed ) );
		if ( sample->m_treeImpact.valid )
			printf( " tree %08x", Bits( sample->m_treeImpact.speed ) );
		g_dump = argc > 2 && step >= atoi( argv[2] ) && step <= atoi( argv[2] ) + 2;
		if ( step % 30 == 29 || g_dump )
		{
			WorldHash wh = { 0xcbf29ce484222325ull, 0 };
			b3AABB box = { { -1000.0f, -1000.0f, -1000.0f }, { 1000.0f, 1000.0f, 1000.0f } };
			b3World_OverlapAABB( sample->m_worldId, box, b3DefaultQueryFilter(), HashShape, &wh );
			printf( " world %d %016llx", wh.count, (unsigned long long)wh.h );
		}
		printf( "\n" );
	}
	return 0;
}
