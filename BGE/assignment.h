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
		
		shared_ptr<PhysicsController> CreateAnimat(glm::vec3 pos);
		assignment(void);
		~assignment(void);
		bool Initialise();
		void Update();
		void CreateWall();
	};
}
