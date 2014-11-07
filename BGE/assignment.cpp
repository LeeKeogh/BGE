#include "assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

assignment::assignment(void)
{
}

assignment::~assignment(void)
{
}

//shared_ptr<PhysicsController> cyl;
//std::shared_ptr<GameComponent> station;

bool assignment::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));

	shared_ptr<PhysicsController> box1[5][5][5];
	
	int i, j,z;
	for (i = 0; i < 5; i++)
	{
		for (j =0 ; j < 5; j++)
		{
			for (z = 0; z < 5; z++)
			box1[i][j][z]=physicsFactory->CreateBox(5, 5, 5, glm::vec3(5*i, 5*j, 5*z), glm::quat());
		}
	}

	if (!Game::Initialise()) {
		return false;
	}



	return true;
}

void BGE::assignment::Update()
{
	
	Game::Update();
}

void BGE::assignment::Cleanup()
{
	Game::Cleanup();
}
