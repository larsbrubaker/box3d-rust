// Scripted Dynamic Mover run; prints one line per step with the mover body state as
// raw float bits so the Rust port can be diffed bit-for-bit.
#include "dyn_sample.h"

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
static bool HashShape( b3ShapeId shapeId, void* context )
{
	WorldHash* wh = (WorldHash*)context;
	b3WorldTransform xf = b3Body_GetTransform( b3Shape_GetBody( shapeId ) );
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
	DynamicMoverSample* sample = new DynamicMoverSample();

	for ( int step = 0; step < stepCount; ++step )
	{
		Keys keys = {};
		float yawDeg = 180.0f;
		if ( 30 <= step && step < 150 )
		{
			keys.w = true;
			keys.d = true;
		}
		if ( ( 160 <= step && step < 170 ) || ( 200 <= step && step < 206 ) )
			keys.space = true;
		if ( 220 <= step && step < 300 )
		{
			yawDeg = 90.0f;
			keys.w = true;
			keys.shift = true;
		}
		if ( 300 <= step && step < 360 )
		{
			yawDeg = 90.0f;
			keys.s = true;
			keys.ctrl = true;
		}
		if ( 600 <= step && step < 700 )
		{
			yawDeg = 0.0f;
			keys.w = true;
		}
		if ( 720 <= step && step < 725 )
			keys.space = true;
		if ( 740 <= step )
		{
			yawDeg = 270.0f;
			keys.w = true;
			keys.d = true;
		}

		sample->m_keys = keys;
		sample->m_camera->m_yaw = yawDeg * DEG_TO_RAD;
		sample->m_camera->m_pitch = 20.0f * DEG_TO_RAD;
		sample->m_camera->UpdateTransform();

		if ( step == 360 )
			sample->MouseDown( 1 );
		if ( step == 372 || step == 384 || step == 396 )
			sample->MouseDown( 0 );
		if ( step == 400 )
			sample->MouseUp( 1 );

		b3BodyId moverId = sample->m_mover.GetBodyId();
		if ( step == 420 )
		{
			b3Pos p = b3Body_GetPosition( sample->m_platformIds[1] );
			b3Pos target = { p.x, 5.4f, p.z };
			b3Body_SetTransform( moverId, target, b3Body_GetRotation( moverId ) );
			b3Body_SetLinearVelocity( moverId, b3Vec3_zero );
		}
		if ( step == 450 || step == 520 )
		{
			sample->m_useBox = !sample->m_useBox;
			sample->SwapMoverShape();
		}
		if ( step == 700 )
			sample->m_mover.SetGravityScale( 3.0f );

		sample->Step();

		b3Pos p = b3Body_GetPosition( moverId );
		b3Vec3 v = b3Body_GetLinearVelocity( moverId );
		int gems = 0;
		for ( b3BodyId id : sample->m_gemIds )
			gems += B3_IS_NON_NULL( id ) ? 1 : 0;
		printf( "%d %08x %08x %08x %08x %08x %08x g%d w%d j%d n%d gems%d plate%d", step, Bits( (float)p.x ),
				Bits( (float)p.y ), Bits( (float)p.z ), Bits( v.x ), Bits( v.y ), Bits( v.z ), (int)sample->m_mover.m_onGround,
				(int)sample->m_mover.m_walkable, (int)sample->m_mover.m_jumping, sample->m_mover.m_probeCount, gems,
				(int)sample->m_plateArmed );
		b3BodyId ball = sample->m_ballIds[0];
		if ( b3Body_IsValid( ball ) )
		{
			b3Pos bp = b3Body_GetPosition( ball );
			printf( " ball %08x %08x %08x", Bits( (float)bp.x ), Bits( (float)bp.y ), Bits( (float)bp.z ) );
		}
		if ( step % 30 == 29 )
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
