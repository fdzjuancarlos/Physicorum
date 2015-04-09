#include "AnimationBlender.h"

AnimationBlender::AnimationBlender(Entity *entity) : mEntity(entity) {
  AnimationStateSet *set = mEntity->getAllAnimationStates();
  AnimationStateIterator it = set->getAnimationStateIterator();
  // Inicializamos los AnimationState de la entidad
  while(it.hasMoreElements()) {
    AnimationState *anim = it.getNext();
    anim->setEnabled(false);
    anim->setWeight(0);
    anim->setTimePosition(0);
  }
  mSource = NULL;  mTarget = NULL;  mTimeleft = 0;
}

void AnimationBlender::blend (const String &animation, 
			      BlendingTransition transition, 
			      Real duration, bool l) {

  AnimationState *newTarget = mEntity->getAnimationState(animation);
  newTarget->setLoop(l);
  mTransition = transition;
  mDuration = duration;
  mLoop = l;
 
  if ((mTimeleft <= 0) || (transition == AnimationBlender::Switch)){
    if (mSource != NULL) mSource->setEnabled(false);
    mSource = newTarget;
    mSource->setEnabled(true);
    mSource->setWeight(1);
    mSource->setTimePosition(0);
    mTimeleft = mSource->getLength(); 
    mTarget = NULL;
  }
  else {
    if (mSource != newTarget) {
      mTarget = newTarget;
      mTarget->setEnabled(true);
      mTarget->setWeight(0);
      mTarget->setTimePosition(0);
    }
  }
}

void AnimationBlender::addTime(Real time) {
  if (mSource == NULL) return;   // No hay fuente
  mSource->addTime(time);   mComplete = false;
  mTimeleft -= time;
  if ((mTimeleft <= 0) && (mTarget == NULL)) mComplete = true;

  if (mTarget != NULL) {  // Si hay destino
    if (mTimeleft <= 0) {  
      mSource->setEnabled(false);  mSource->setWeight(0);
      mSource = mTarget;
      mSource->setEnabled(true);   mSource->setWeight(1);
      mTimeleft = mSource->getLength();
      mTarget = NULL;
    }
    else {   // Queda tiempo en Source... cambiar pesos
      Real weight = mTimeleft / mDuration;
      if (weight > 1) weight = 1.0;
      mSource->setWeight(weight);
      mTarget->setWeight(1.0 - weight);
      if (mTransition == AnimationBlender::Blend) mTarget->addTime(time);
    }
  }
  if ((mTimeleft <= 0) && mLoop) mTimeleft = mSource->getLength();
}


