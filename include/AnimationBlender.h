#include <Ogre.h>
using namespace Ogre;

class AnimationBlender {
 public:
   enum BlendingTransition {
     Switch,  // Parar fuente y reproduce destino
     Blend    // Cross fade (Mezclado suave)
   };
   
 private:
   Entity *mEntity;                 // Entidad sobre la que mezclamos
   AnimationState *mSource;         // Animacion inicio de la mezcla
   AnimationState *mTarget;         // Animacion destino
   BlendingTransition mTransition;  // Tipo de transicion
   bool mLoop;                      // Animacion en bucle?
   
 public: 
   Real mTimeleft;    // Tiempo restante de la animacion (segundos)
   Real mDuration;    // Tiempo invertido en el mezclado (segundos)
   bool mComplete;    // Ha finalizado la animacion?
   
   AnimationBlender( Entity *);
   void blend(const String &animation, BlendingTransition transition, 
	       Real duration, bool l=true);
   void addTime(Real);
   Real getProgress() { return mTimeleft/mDuration; }
   AnimationState *getSource() { return mSource; }
   AnimationState *getTarget() { return mTarget; }
   bool getLoop() { return mLoop; }
};
