

	void MouseDown( int button )
	{
		if ( m_camera->m_thirdPerson )
		{
			if ( button == 1 )
				m_aiming = true;
			else if ( button == 0 && m_aiming )
				m_shootRequested = true;
		}
	}

	void MouseUp( int button )
	{
		if ( button == 1 )
			m_aiming = false;
	}

	// DynamicMoverSample::Step minus FollowMover / drawing (camera + rendering only).
	void Step()
	{
		bool pause = false;
		float timeStep = 1.0f / 60.0f;

		if ( pause == false && timeStep > 0.0f )
		{
			m_time += timeStep;

			for ( int i = 0; i < m_platformCount; ++i )
			{
				b3WorldTransform target = { GetPlatformPosition( i, m_time ), b3Quat_identity };
				b3Body_SetTargetTransform( m_platformIds[i], target, timeStep, true );
			}

			MoverInput input = GetMoverInput();

			// Jumping again needs a fresh press
			if ( input.jump )
			{
				if ( m_jumpReleased && m_mover.Jump() )
				{
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			float speedScale = 1.0f;
			if ( input.walk || m_aiming )
			{
				speedScale = 0.25f;
			}
			else if ( input.sprint )
			{
				speedScale = 1.5f;
			}
			m_mover.Update( timeStep, input.forward, input.right, input.throttle, speedScale );
		}

		// Shoot before the step so the ball is in the world when it is drawn
		if ( m_aiming && m_shootRequested )
		{
			float leadTime = pause == false ? timeStep : 0.0f;
			ShootBall( ComputeAim(), leadTime );
		}

		m_shootRequested = false;

		m_didStep = false;
		b3World_Step( m_worldId, timeStep, 4 );
		m_didStep = true;

		if ( m_didStep )
		{
			CollectGems();
			UpdatePressurePlate();
		}

		m_mover.Draw();
	}

	static constexpr uint64_t StaticBit = 0x1;
	static constexpr uint64_t MoverBit = 0x2;
	static constexpr uint64_t GemBit = 0x4;
	static constexpr uint64_t DynamicBit = 0x10;
	static constexpr uint64_t ProjectileBit = 0x20;
	static constexpr uint64_t DebrisBit = 0x40;
	static constexpr uint64_t AllBits = ~0ull;

	static constexpr int m_ballCapacity = 50;

	static constexpr float m_launcherRadius = 0.03f;
	static constexpr float m_launcherBack = 0.05f;
	static constexpr float m_launcherFront = 0.55f;
	static constexpr float m_ballRadius = 0.1f;

	static constexpr uint64_t m_platformMaterial = 7;
	static constexpr int m_platformCount = 2;
	static constexpr int m_pillarCount = 6;
	static constexpr int m_gemCount = 17;
	static constexpr b3Pos m_sweeperCenter = { 0.0f, 0.0f, -16.0f };
	static constexpr b3Pos m_plateCenter = { 0.0f, 0.05f, 26.5f };
	static constexpr float m_plateTravel = 0.1f;

	DynamicMover m_mover;
	b3JointId m_plateJointId;
	bool m_plateArmed;
	b3BodyId m_platformIds[m_platformCount];
	b3BodyId m_gemIds[m_gemCount];
	float m_time;
	bool m_jumpReleased;
	bool m_drawProbes;
	bool m_useBox;

	b3BodyId m_ballIds[m_ballCapacity];
	int m_ballIndex;
	float m_ballSpeed;
	bool m_aiming;
	float m_aimBlend;
	bool m_shootRequested;
	b3HullData* m_launcherHull;
};
