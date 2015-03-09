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

#ifndef GameState_H
#define GameState_H

#include <Ogre.h>
#include <OIS/OIS.h>
 
#include "GameManager.h"
#include "InputManager.h"

// Clase abstracta de estado básico.
// Definición base sobre la que extender
// los estados del juego.
class GameState {

 public:
  GameState() {}
  
  // Gestión básica del estado.
  virtual void enter () = 0;
  virtual void exit () = 0;
  virtual void pause () = 0;
  virtual void resume () = 0;

  // Gestión básica para el tratamiento
  // de eventos de teclado y ratón.
  virtual void keyPressed (const OIS::KeyEvent &e) = 0;
  virtual void keyReleased (const OIS::KeyEvent &e) = 0;

  virtual void mouseMoved (const OIS::MouseEvent &e) = 0; 
  virtual void mousePressed (const OIS::MouseEvent &e, 
			     OIS::MouseButtonID id) = 0;
  virtual void mouseReleased (const OIS::MouseEvent &e, 
			      OIS::MouseButtonID id) = 0;

  // Gestión básica para la gestión
  // de eventos antes y después de renderizar un frame.
  virtual bool frameStarted (const Ogre::FrameEvent& evt) = 0;
  virtual bool frameEnded (const Ogre::FrameEvent& evt) = 0;
	
  // Gestión básica de transiciones.
  void changeState (GameState* state) { 
    GameManager::getSingletonPtr()->changeState(state);
  }
  void pushState (GameState* state) {
    GameManager::getSingletonPtr()->pushState(state);
  }
  void popState () {
    GameManager::getSingletonPtr()->popState();
  }

};

#endif
