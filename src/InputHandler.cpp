#include "InputHandler.h"

using namespace Ogre;

void InputHandler::keyPressed (const OIS::KeyEvent &e, Ogre::Camera* camera){

  if (e.key == OIS::KC_W) {
    Vector3 cameraDirection = camera->getDirection();
    Vector3 movement = Vector3(cameraDirection.x,0,cameraDirection.z);
    camera->move(movement);
  }

}
