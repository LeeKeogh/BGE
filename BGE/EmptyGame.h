#pragma once
#include "Game.h"

namespace BGE
{
	class EmptyGame :
		public Game
	{
	public:
		shared_ptr<PhysicsController> CreateAnimat(glm::vec3 pos);
		EmptyGame();
		~EmptyGame();

		bool Initialise();
		void Update();
	};
}
