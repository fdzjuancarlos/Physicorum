#include "AnimationUpdater.h"

using namespace Ogre;

void AnimationUpdater::keyPressed (const OIS::KeyEvent &e){
  if(_input){
    if (e.key == OIS::KC_W){ 
	_forward=true;
	_transform=false;
    }
    if (e.key == OIS::KC_S){ 
	_back=true;
	_transform=false;
    }
  }
  _input=true;
}

void AnimationUpdater::keyReleased (const OIS::KeyEvent &e){
  if (e.key == OIS::KC_W) _forward=false;
  if (e.key == OIS::KC_S) _back=false;
  if (e.key == OIS::KC_E){
	_transform=true;
	_input=false;
	if(_ball){
		_animBlender->blend("Rtransform", AnimationBlender::Blend, 6, false);
		_ball=false;
	}else{
		_animBlender->blend("Btransform", AnimationBlender::Blend, 6, false);
		_ball=true;
  	}
  }
}

void AnimationUpdater::update(const Ogre::FrameEvent& evt){

  if(_ball==false){
	if(_forward==false && _back==false && _transform==false){
		_animBlender->blend("Stop", AnimationBlender::Switch, 0, false);
  	}
	if(_forward){
		_animBlender->blend("Walk", AnimationBlender::Blend, 0.5, false);
	}
	if(_back){
		_animBlender->blend("Moonwalk", AnimationBlender::Blend, 0.5, false);
	}
  }else{
	if(_forward==false && _back==false && _transform==false){
		_animBlender->blend("Bstop", AnimationBlender::Switch, 0, false);
  	}
	if(_forward){
		_animBlender->blend("Bwalk", AnimationBlender::Blend, 0.5, false);
	}
	if(_back){
		_animBlender->blend("Bmoonwalk", AnimationBlender::Blend, 0.5, false);
	}
  }

	_animBlender->addTime(evt.timeSinceLastFrame);
}
