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


EmptyGame::EmptyGame()
{
}


EmptyGame::~EmptyGame()
{
}

bool BGE::EmptyGame::Initialise()
{
	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	CreateAnimat(glm::vec3(20, 20, 10));
	physicsFactory->CreateCameraPhysics();
	physicsFactory->CreateGroundPhysics();
	return Game::Initialise();
}
shared_ptr<PhysicsController> EmptyGame::CreateAnimat(glm::vec3 position)
{
	
	float width = 18;
	float height = 2;
	float length = 1;
	float wheelWidth = 3;
	float wheelRadius = 5;
	float wheelOffset = 3.0f;

	//shared_ptr<PhysicsController> chassis = physicsFactory->CreateBox(width, height, length, position, glm::quat());
	 
	shared_ptr<PhysicsController> chassis2 = physicsFactory->CreateBox(width, height, length, glm::vec3(position.x, position.y, position.z) , glm::quat());

	shared_ptr<PhysicsController> chassis3 = physicsFactory->CreateBox(18, 4, 6, glm::vec3(position.x, position.y, position.z+2), glm::quat());
	shared_ptr<PhysicsController> chassis4 = physicsFactory->CreateBox(1, 8, 1, glm::vec3(position.x, position.y+5, position.z+2), glm::quat());
	shared_ptr<PhysicsController> chassis5 = physicsFactory->CreateBox(1, 8, 1, glm::vec3(position.x,position.y+5, position.z+4), glm::quat());

	shared_ptr<PhysicsController> beam = physicsFactory->CreateBox(30, 1, 1, glm::vec3(position.x, position.y+15, position.z+2), glm::quat());

	shared_ptr<PhysicsController> beam2 = physicsFactory->CreateBox(1, 10, 1, glm::vec3(position.x, position.y+10, position.z+2), glm::quat());

	shared_ptr<PhysicsController> beam3 = physicsFactory->CreateBox(1, 5, 1, glm::vec3(position.x, position.y+20, position.z+2), glm::quat());

	shared_ptr<PhysicsController> wheel;
	shared_ptr<PhysicsController> wheel2;
	shared_ptr<PhysicsController> wheel3;
	glm::quat q = glm::angleAxis(glm::half_pi<float>(), glm::vec3(1, 0, 0));

	glm::vec3 offset;
	btHingeConstraint * hinge;

	//offset = glm::vec3(-(width / 2 - 2), 0, -(length / 2 + wheelOffset));
	wheel = physicsFactory->CreateCylinder(wheelRadius, wheelWidth, position + offset, q);
	//hinge = new btHingeConstraint(*chassis->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	//dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(-(width / 2 - 2), 0, +(length / 2 + wheelOffset));
	hinge = new btHingeConstraint(*chassis2->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
//	hinge->enableAngularMotor(true, 10, 10);
	dynamicsWorld->addConstraint(hinge);


	//offset = glm::vec3(+(width / 2 - 2), 0, +(length / 2 + wheelOffset));
	//hinge = new btHingeConstraint(*chassis2->rigidBody, *chassis3->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	//dynamicsWorld->addConstraint(hinge);

	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(14, 0, 4));
	//t2.setRotation(GLToBtQuat(glm::angleAxis(-90.0f, glm::vec3(0, 1, 0)))); // Not sure but this needs to be a minus angle
	t2.setOrigin(btVector3(2, 0, 0));
	btFixedConstraint * fixed = new btFixedConstraint(*chassis2->rigidBody, *chassis3->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(fixed);

	offset = glm::vec3(+(width / 2 - 2), 0, +(length / 2 + wheelOffset+3));
	wheel2 = physicsFactory->CreateCylinder(6, 5, glm::vec3(position) + offset, q);
	hinge = new btHingeConstraint(*chassis3->rigidBody, *wheel2->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(+(width / 2 - 2), 0, -(length / 2 + wheelOffset + 3));
	wheel2 = physicsFactory->CreateCylinder(6, 5, glm::vec3(position) + offset, q);
	hinge = new btHingeConstraint(*chassis3->rigidBody, *wheel2->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);

	btTransform t3, t4;
	t3.setIdentity();
	t4.setIdentity();
	t3.setOrigin(btVector3(0, 1, 0));
	t4.setOrigin(btVector3(7, -5, 0));
	btFixedConstraint * fixed2 = new btFixedConstraint(*chassis3->rigidBody, *chassis4->rigidBody, t3, t4);
	dynamicsWorld->addConstraint(fixed2);

	offset = glm::vec3(-3,4, 2);
	hinge = new btHingeConstraint(*chassis4->rigidBody, *beam->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(-14, -2, 0);
	hinge = new btHingeConstraint(*beam->rigidBody, *beam2->rigidBody, GLToBtVector(offset), btVector3(0, -6, 0), btVector3(0, 1, 0), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(hinge);


	offset = glm::vec3(0, 4, -(length / 2 + wheelOffset)+1);
	hinge = new btHingeConstraint(*beam2->rigidBody, *wheel->rigidBody, GLToBtVector(offset), btVector3(0, 0, 3), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);

	btTransform t5, t6;
	t5.setIdentity();
	t6.setIdentity();
	t5.setOrigin(btVector3(0, 1, 0));
	t6.setOrigin(btVector3(3, -5, 2));
	btFixedConstraint * fixed3 = new btFixedConstraint(*chassis3->rigidBody, *chassis5->rigidBody, t5, t6);
	dynamicsWorld->addConstraint(fixed3);

	offset = glm::vec3(0, -.5, 1.5);
	wheel3 = physicsFactory->CreateCylinder(2, 1, glm::vec3(position) + offset, q);
	hinge = new btHingeConstraint(*chassis5->rigidBody, *wheel3->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(7, -2, 0);
	hinge = new btHingeConstraint(*beam->rigidBody, *beam3->rigidBody, GLToBtVector(offset), btVector3(0, -3, 0), btVector3(0, 1, 0), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(0, 2.5, -(length / 2 + wheelOffset-1));
	hinge = new btHingeConstraint(*beam3->rigidBody, *wheel3->rigidBody, GLToBtVector(offset), btVector3(0, 0, 2), btVector3(0, 0, 1), btVector3(0, 1, 0), true);
	hinge->enableAngularMotor(true, 20, 20);
	dynamicsWorld->addConstraint(hinge);



	return chassis2;



}
void EmptyGame::Update()
{
	Game::Update();
}
