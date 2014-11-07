#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class assignment :
		public Game
	{
	private:

	public:
		assignment(void);
		~assignment(void);
		bool Initialise();
		void Update();
		void Cleanup();
		void CreateWall();
	};
}
