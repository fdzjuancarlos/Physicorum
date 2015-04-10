
/*********************************************************************
 * Módulo 1. Curso de Experto en Desarrollo de Videojuegos
 * Autor: David Vallejo Fernández    David.Vallejo@uclm.es
 *
 * Código modificado a partir de Managing Game States with OGRE
 * http://www.ogre3d.org/tikiwiki/Managing+Game+States+with+OGRE
 * Inspirado en Managing Game States in C++
 * http://gamedevgeek.com/tutorials/managing-game-states-in-c/
 *
 * You can redistribute and/or modify this file under the terms of the
 * GNU General Public License ad published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * and later version. See <http://www.gnu.org/licenses/>.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.  
 *********************************************************************/

#ifndef AnimationUpdater_H
#define AnimationUpdater_H

#include <Ogre.h>
#include <OIS/OIS.h>
 
#include "GameManager.h"
#include "InputManager.h"
#include "AnimationBlender.h"

class AnimationUpdater{

  std::shared_ptr<SceneNode> _player;
  //std::shared_ptr<AnimationBlender> _animBlender;
  AnimationBlender *_animBlender;
  //Player events
  bool _forward;
  bool _back;
  bool _transform;
  bool _ball;
  bool _input;

 public:
  
  
  AnimationUpdater(std::shared_ptr<SceneNode> player){
    _player = player;
    auto sceneMgr = Root::getSingletonPtr()->getSceneManager("SceneManager");
    //_animBlender->AnimationBlender(sceneMgr->getEntity("Robotillo"));
    _animBlender = new AnimationBlender(sceneMgr->getEntity("Robotillo"));
    _forward=false;
    _back=false; 
    _transform=false;
    _ball=false;
    _input=true;
  };
  
  void keyPressed (const OIS::KeyEvent &e);
  void keyReleased (const OIS::KeyEvent &e);

  void update(const Ogre::FrameEvent& evt);
  void transform(const Ogre::FrameEvent& evt);
};

#endif
