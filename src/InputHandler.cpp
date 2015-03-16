#include "InputHandler.h"

using namespace Ogre;


void InputHandler::keyPressed (const OIS::KeyEvent &e, Ogre::Camera* camera){

  if (e.key == OIS::KC_W) {
    Vector3 cameraDirection = camera->getDirection();
    Vector3 movement = Vector3(cameraDirection.x,0,cameraDirection.z);
    camera->move(movement); 
  }

}




void InputHandler::mouseMoved(const OIS::MouseEvent &e){
  float rotx = e.state.X.rel * 0.01* -1;
  float roty = e.state.Y.rel * 0.01* -1;
  _camera->yaw(Ogre::Radian(rotx));
  _camera->pitch(Ogre::Radian(roty));
}
