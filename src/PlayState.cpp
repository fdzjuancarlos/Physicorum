#include "PlayState.h"
#include "PauseState.h"

using namespace Ogre;

template<> PlayState* Ogre::Singleton<PlayState>::msSingleton = 0;

void
PlayState::enter ()
{
	_root = Ogre::Root::getSingletonPtr();

	// Se recupera el gestor de escena y la cámara.
	_sceneMgr = _root->getSceneManager("SceneManager");
	_camera = _sceneMgr->getCamera("IntroCamera");
	_viewport = _root->getAutoCreatedWindow()->addViewport(_camera);
	
	//Camera configuration
//	_camera->lookAt(Vector3(0, -15, -30));
	_camera->setNearClipDistance(0.1);
	_camera->setFarClipDistance(100);

	// Nuevo background colour.
	_viewport->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0));
  
	//Paddle Initialization
	Ogre::Entity* ent1 = _sceneMgr->createEntity("RobotilloMesh.mesh");
	ent1->setQueryFlags(PLAYER);
  std::shared_ptr<SceneNode> player(_sceneMgr->createSceneNode("playerPaddle"));
	_player = player;
	_player->attachObject(ent1);
	_sceneMgr->getRootSceneNode()->addChild(_player.get());
	_player->setScale(1,1,1);
	_player->setPosition(0,-30,-40); 

  _inputHandler = std::make_shared<InputHandler>(_camera,_player);
  //Ground and Lights initialization
  _sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);	
  _sceneMgr->setShadowColour(Ogre::ColourValue(0.5, 0.5, 0.5) );
  _sceneMgr->setAmbientLight(Ogre::ColourValue(0.9, 0.9, 0.9));

  _sceneMgr->setShadowTextureCount(2);
  _sceneMgr->setShadowTextureSize(512);
  
  Ogre::Light* light = _sceneMgr->createLight("Light1");
  light->setPosition(-5,12,2);
  light->setType(Ogre::Light::LT_SPOTLIGHT);
  light->setDirection(Ogre::Vector3(1,-1,0));
  light->setSpotlightInnerAngle(Ogre::Degree(25.0f));
  light->setSpotlightOuterAngle(Ogre::Degree(60.0f));
  light->setSpotlightFalloff(5.0f);
  light->setCastShadows(true);

  Ogre::Light* light2 = _sceneMgr->createLight("Light2");
  light2->setPosition(3,12,3);
  light2->setDiffuseColour(0.2,0.2,0.2);
  light2->setType(Ogre::Light::LT_SPOTLIGHT);
  light2->setDirection(Ogre::Vector3(-0.3,-1,0));
  light2->setSpotlightInnerAngle(Ogre::Degree(25.0f));
  light2->setSpotlightOuterAngle(Ogre::Degree(60.0f));
  light2->setSpotlightFalloff(10.0f);
  light2->setCastShadows(true);

  //Ogre::Plane plane1(Ogre::Vector3::UNIT_Y, -50);
  //Ogre::MeshManager::getSingleton().createPlane("plane1",
	//Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane1,
	//200,200,1,1,true,1,20,20,Ogre::Vector3::UNIT_Z);

  //Node* node2 = _sceneMgr->createSceneNode("ground");
  //Ogre::Entity* groundEnt = _sceneMgr->createEntity("planeEnt", "plane1");
  //groundEnt->setMaterialName("Ground");
  //groundEnt->setCastShadows(false);
 // node2->attachObject(groundEnt);
 // _sceneMgr->getRootSceneNode()->addChild(node2);
	_exitGame = false;

  //=============PHYSICS===========//
  // Creacion del mundo (definicion de los limites y la gravedad) ---
  AxisAlignedBox worldBounds = AxisAlignedBox (
    Vector3 (-10000, -10000, -10000), 
    Vector3 (10000,  10000,  10000));
  Vector3 gravity = Vector3(0, -9.8, 0);

  _world = new OgreBulletDynamics::DynamicsWorld(_sceneMgr,
 	   worldBounds, gravity);
  _world->setShowDebugShapes (true);  // Muestra los collision shapes

  // Creacion de los elementos iniciales del mundo

  // Creacion de la entidad y del SceneNode ------------------------
  Plane plane1(Vector3(0,1,0), -50);    // Normal y distancia
  MeshManager::getSingleton().createPlane("plane1",
	ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane1,
	200, 200, 1, 1, true, 1, 20, 20, Vector3::UNIT_Z);
  SceneNode* node = _sceneMgr->createSceneNode("ground");
  Entity* groundEnt = _sceneMgr->createEntity("planeEnt", "plane1");
  groundEnt->setMaterialName("Ground");
  node->attachObject(groundEnt);
  _sceneMgr->getRootSceneNode()->addChild(node);

  // Creamos forma de colision para el plano ----------------------- 
  OgreBulletCollisions::CollisionShape *Shape;
  Shape = new OgreBulletCollisions::StaticPlaneCollisionShape
   (Ogre::Vector3(0,1,0), -50);   // Vector normal y distancia
  OgreBulletDynamics::RigidBody *rigidBodyPlane = new 
  OgreBulletDynamics::RigidBody("rigidBodyPlane", _world);

  // Creamos la forma estatica (forma, Restitucion, Friccion) ------
  rigidBodyPlane->setStaticShape(Shape, 0.1, 0.8); 
  
  // Anadimos los objetos Shape y RigidBody ------------------------
 // _shapes.push_back(Shape);
  _bodies.push_back(rigidBodyPlane);

}

void
PlayState::exit ()
{
  _sceneMgr->clearScene();
  _root->getAutoCreatedWindow()->removeAllViewports();
}

void
PlayState::pause()
{
}

void
PlayState::resume()
{
  // Se restaura el background colour.
  _viewport->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 1.0));
}

bool
PlayState::frameStarted
(const Ogre::FrameEvent& evt)
{
  _lastTime= evt.timeSinceLastFrame;
  _world->stepSimulation(_lastTime); // Actualizar simulacion Bullet
  _inputHandler->update(evt,_player->getPosition());
  return true;
}

bool
PlayState::frameEnded
(const Ogre::FrameEvent& evt)
{
  if (_exitGame)
    return false;
  
  Real deltaT = evt.timeSinceLastFrame;
  _world->stepSimulation(deltaT); // Actualizar simulacion Bullet
  return true;
}

void
PlayState::keyPressed
(const OIS::KeyEvent &e)
{
  // Tecla p --> PauseState.
  if (e.key == OIS::KC_P) {
    pushState(PauseState::getSingletonPtr());
    _exitGame = true;
  }
  _inputHandler->keyPressed(e);
}

void
PlayState::keyReleased
(const OIS::KeyEvent &e)
{
  if (e.key == OIS::KC_ESCAPE) {
   // _exitGame = true;
   
  Vector3 size = Vector3::ZERO;	// size of the box
  // starting position of the box
  Vector3 position = (_camera->getDerivedPosition() 
     + _camera->getDerivedDirection().normalisedCopy() * 10);
 
  Entity *entity = _sceneMgr->createEntity("Box" + 
     StringConverter::toString(2), "cube.mesh");
  entity->setMaterialName("cube");
  SceneNode *node = _sceneMgr->getRootSceneNode()->
    createChildSceneNode();
  node->attachObject(entity);

  // Obtenemos la bounding box de la entidad creada... ------------
  AxisAlignedBox boundingB = entity->getBoundingBox();
  size = boundingB.getSize(); 
  size /= 2.0f;   // El tamano en Bullet se indica desde el centro
 
  // after that create the Bullet shape with the calculated size
  OgreBulletCollisions::BoxCollisionShape *boxShape = new 
    OgreBulletCollisions::BoxCollisionShape(size);
 // and the Bullet rigid body
  OgreBulletDynamics::RigidBody *rigidBox = new 
    OgreBulletDynamics::RigidBody("rigidBox" + 
       StringConverter::toString(2), _world);

  rigidBox->setShape(node, boxShape,
		     0.6 /* Restitucion */, 0.6 /* Friccion */,
		     5.0 /* Masa */, position /* Posicion inicial */,
		     Quaternion(0,0,0,1) /* Orientacion */);
/*
  rigidBox->setLinearVelocity(
     _camera->getDerivedDirection().normalisedCopy() * 7.0); 
*/

  // Anadimos los objetos a las deques
//  _bodies.push_back(rigidBox);
  }
  _inputHandler->keyReleased(e);
}

void
PlayState::mouseMoved
(const OIS::MouseEvent &e)
{
  _inputHandler->mouseMoved(e);
}

void
PlayState::mousePressed
(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
}

void
PlayState::mouseReleased
(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
}

PlayState*
PlayState::getSingletonPtr ()
{
return msSingleton;
}

PlayState&
PlayState::getSingleton ()
{ 
  assert(msSingleton);
  return *msSingleton;
}

double PlayState::getTimeSinceLastTime(){
  return _lastTime;
}


