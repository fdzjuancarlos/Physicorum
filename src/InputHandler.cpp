#include "InputHandler.h"

using namespace Ogre;


void InputHandler::keyPressed (const OIS::KeyEvent &e){
  if(_input){
    if (e.key == OIS::KC_W) _forward=true;
    if (e.key == OIS::KC_S) _back=true;
    if (e.key == OIS::KC_A) _left=true;
    if (e.key == OIS::KC_D) _right=true;
  }
  _input=true;
}

void InputHandler::keyReleased (const OIS::KeyEvent &e){
  if (e.key == OIS::KC_W) _forward=false;
  if (e.key == OIS::KC_S) _back=false;
  if (e.key == OIS::KC_A) _left=false;
  if (e.key == OIS::KC_D) _right=false;
  if (e.key == OIS::KC_E) _input=false;
}

void InputHandler::update(const Ogre::FrameEvent& evt, Vector3 target){

  Vector3 cameraPosition = _player->convertLocalToWorldPosition(Vector3(0,20,20));
  Vector3 pointToTarget = _player->convertLocalToWorldPosition(Vector3(0,15,10));

  _extendedCamera->update(evt.timeSinceLastFrame,cameraPosition,pointToTarget);

  double unitOfTime = evt.timeSinceLastFrame * 10;

  Vector3 forward(0,0,-unitOfTime);
  Vector3 back(0,0,unitOfTime);
  Vector3 left(-unitOfTime,0,0);
  Vector3 right(unitOfTime,0,0);
  Vector3 result(0,0,0);
  if(_forward) result += forward;
  if(_back) result += back;
  if(_left)  _player->yaw(Radian(2*evt.timeSinceLastFrame));
  if(_right) _player->yaw(Radian(-2*evt.timeSinceLastFrame)); 
  _player->translate(result, Node::TS_LOCAL);

}


void InputHandler::mouseMoved(const OIS::MouseEvent &e){
  float rotx = e.state.X.rel * 0.01* -1;
  float roty = e.state.Y.rel * 0.01* -1;
  _camera->yaw(Ogre::Radian(rotx));
  _camera->pitch(Ogre::Radian(roty));
}
