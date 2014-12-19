#include "assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"

#include "EmptyGame.h"
#include "PhysicsFactory.h"
#include "Game.h"
#include "Sphere.h"
#include "Box.h"
#include "Cylinder.h"
#include "Ground.h"
#include "Content.h"
#include "PhysicsCamera.h"
#include "Model.h"
#include "dirent.h"
#include "Utils.h"
#include "Capsule.h"
using namespace BGE;




assignment::assignment()
{
}


assignment::~assignment()
{
}

bool BGE::assignment::Initialise()
{
	
	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	physicsFactory->CreateCameraPhysics();
	physicsFactory->CreateGroundPhysics();
	Update();
	physicsFactory->CreateAnimat(glm::vec3(5, 10, 10));
//	for (int i = 0; i < 1000; i += 200)
//	{
//		for (int j = 0; j < 1000; j += 200){
//			physicsFactory->CreateTree(glm::vec3(i, 0, j));
//		}
//	}

	
	return Game::Initialise();
}

void BGE::assignment::Update()
{
	
	Game::Update();
}

