
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

#ifndef InputHandler_H
#define InputHandler_H

#define THIRD_PERSON 1
#define FIRST_PERSON 2

#include <Ogre.h>
#include <OIS/OIS.h>
 
#include "GameManager.h"
#include "InputManager.h"

class InputHandler{

  int _inputMode;

 public:
  
  InputHandler() : _inputMode(FIRST_PERSON){};
  
  void keyPressed (const OIS::KeyEvent &e, Ogre::Camera* camera);
  void keyReleased (const OIS::KeyEvent &e, Ogre::Camera* camera);

};

#endif
