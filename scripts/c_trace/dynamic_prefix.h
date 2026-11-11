// Headless trace driver for the Character / Dynamic Mover sample at upstream 3362633.
// The scene construction and the physics methods are copied verbatim from
// samples/sample_character.cpp (DynamicMoverSample); only the app plumbing (camera,
// keys, ImGui, rendering) is replaced by a scripted camera + key table.
#include "dynamic_mover.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <initializer_list>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static constexpr float HALF_PI = B3_PI * 0.5f;
static constexpr float DEG_TO_RAD = B3_PI / 180.0f;
static constexpr float PITCH_LIM = HALF_PI - 0.01f;

static b3Vec3 ForwardFromAngles( float yaw, float pitch )
{
	const float cp = cosf( pitch );
	return b3Vec3{ sinf( yaw ) * cp, sinf( pitch ), cosf( yaw ) * cp };
}

// The host Camera members the sample touches (host/camera.cpp RebuildBasisAndView).
struct Camera
{
	b3Pos m_pivot = {};
	float m_yaw = 0.0f, m_pitch = 0.0f, m_radius = 25.0f;
	bool m_thirdPerson = false;
	b3Pos m_worldEye = {};
	b3Vec3 m_right = { 1, 0, 0 }, m_up = { 0, 1, 0 }, m_forward = { 0, 0, 1 };

	void UpdateTransform()
	{
		const b3Vec3 forward = ForwardFromAngles( m_yaw, m_pitch );
		const b3Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
		const b3Vec3 upUnnorm = b3Sub( worldUp, b3MulSV( b3Dot( worldUp, forward ), forward ) );
		const b3Vec3 up = b3Normalize( upUnnorm );
		const b3Vec3 right = b3Normalize( b3Cross( up, forward ) );
		m_worldEye = b3OffsetPos( m_pivot, b3MulSV( m_radius, forward ) );
		m_right = right;
		m_up = up;
		m_forward = forward;
	}
	void SetView( float yawDegrees, float pitchDegrees, float radius, b3Pos pivot )
	{
		m_pivot = pivot;
		m_yaw = yawDegrees * DEG_TO_RAD;
		m_pitch = b3ClampFloat( pitchDegrees * DEG_TO_RAD, -PITCH_LIM, PITCH_LIM );
		m_radius = b3ClampFloat( radius, 0.1f, 1000.0f );
		UpdateTransform();
	}
	b3Vec3 GetForward() const { return m_forward; }
	b3Vec3 GetRight() const { return m_right; }
};

struct MoverInput
{
	b3Vec3 forward;
	b3Vec3 right;
	b3Vec2 throttle;
	bool jump;
	bool sprint;
	bool walk;
};

struct Keys
{
	bool w, a, s, d, space, shift, ctrl;
};

class DynamicMoverSample
{
public:
	static constexpr bool m_isDebug = true;
	b3WorldId m_worldId;
	Camera m_cam;
	Camera* m_camera = &m_cam;
	Keys m_keys = {};
	bool m_didStep = false;

	b3BodyId AddGroundBox( float extent )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.name = "ground";
		bodyDef.position = { 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull hull = b3MakeBoxHull( extent, 1.0f, extent );
		b3CreateHullShape( groundId, &shapeDef, &hull.base );
		return groundId;
	}

	// Sample::GetMoverInput (sample.cpp)
	MoverInput GetMoverInput() const
	{
		MoverInput input = {};
		input.forward = -m_camera->GetForward();
		input.right = m_camera->GetRight();

		input.forward.y = 0.0f;
		input.forward = b3Normalize( input.forward );

		if ( m_camera->m_thirdPerson == false )
		{
			return input;
		}

		if ( m_keys.w )
			input.throttle.x += 1.0f;
		if ( m_keys.s )
			input.throttle.x -= 1.0f;
		if ( m_keys.a )
			input.throttle.y -= 1.0f;
		if ( m_keys.d )
			input.throttle.y += 1.0f;

		input.jump = m_keys.space;
		input.sprint = m_keys.shift;
		input.walk = m_keys.ctrl;
		return input;
	}

	DynamicMoverSample()
	{
		// Sample::CreateWorld with the default SampleContext
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.workerCount = 1;
		worldDef.enableSleep = true;
		m_worldId = b3CreateWorld( &worldDef );
		b3World_SetContactRecycleDistance( m_worldId, 0.05f );

