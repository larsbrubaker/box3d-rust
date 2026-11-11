// Stub of samples/gfx/draw.h for the headless trace driver: dynamic_mover.cpp only
// draws debug lines / points, which do not affect the simulation.
#pragma once
#include "box3d/collision.h"
#include "box3d/types.h"

struct Vec4
{
	float x, y, z, w;
};

inline Vec4 MakeColor( b3HexColor c )
{
	return { ( ( c >> 16 ) & 0xff ) / 255.0f, ( ( c >> 8 ) & 0xff ) / 255.0f, ( c & 0xff ) / 255.0f, 1.0f };
}

inline void DrawLine( b3Pos, b3Pos, Vec4 )
{
}

inline void DrawPoint( b3Pos, float, Vec4 )
{
}

inline void DrawSolidCapsule( b3WorldTransform, b3Capsule, Vec4 )
{
}
