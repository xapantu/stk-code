//  $Id$
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2008 Joerg Henrichs
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_SFX_MANAGER_HPP
#define HEADER_SFX_MANAGER_HPP

#include <vector>
#ifdef __APPLE__
#  include <OpenAL/al.h>
#else
#  include <AL/al.h>
#endif

#include "lisp/lisp.hpp"
#include "utils/vec3.hpp"

class SFXBase;


/** Manager of all sound effects. The manager reads all sound effects and
 *  maintains the corresponding buffers. Each sound effect objects uses
 *  on of the (shared) buffers from the sound manager.
 */
class SFXManager
{
public:
    /** The different type of sound effects. */
    enum SFXType
    {
        SOUND_UGH,  SOUND_SKID, SOUND_BOWLING_ROLL, SOUND_BOWLING_STRIKE, SOUND_WINNER, SOUND_CRASH, SOUND_GRAB,
		SOUND_SHOT, SOUND_GOO, SOUND_WEE, SOUND_EXPLOSION, SOUND_BZZT, SOUND_BEEP, SOUND_BACK_MENU, SOUND_USE_ANVIL,
        SOUND_USE_PARACHUTE, SOUND_SELECT_MENU, SOUND_MOVE_MENU, SOUND_FULL,
        SOUND_PRESTART, SOUND_START, SOUND_ENGINE_SMALL, SOUND_ENGINE_LARGE,
        NUM_SOUNDS
    };


    /*
        Entries for custom SFX sounds.  These are unique for each kart.
        eg. kart->playCustomSFX(SFX_MANAGER::CUSTOM_HORN)
    */
    enum CustomSFX
    {
        CUSTOM_HORN,    // Replaces default horn
        CUSTOM_CRASH,   // Played when colliding with another kart
        CUSTOM_WIN,     // Played when racer wins
        CUSTOM_EXPLODE, // Played when struck by bowling ball or dynamite
        CUSTOM_GOO,     // Played when driving through goo
        CUSTOM_PASS,    // Played when passing another kart
        CUSTOM_ZIPPER,  // Played when kart hits zipper
        CUSTOM_NAME,    // Introduction "I'm Tux!"
        CUSTOM_ATTACH,  // Played when something is attached to kart (Uh-Oh)
        CUSTOM_SHOOT,   // Played when weapon is used
        NUM_CUSTOMS
    };

    // LISP (or in the future xml) tag for each custom sound
    const char *getCustomTagName(int id);


    /** Status of a sound effect. */
    enum SFXStatus
    {
        SFX_UNKNOWN = -1, SFX_STOPED = 0, SFX_PAUSED = 1, SFX_PLAYING = 2,
        SFX_INITIAL = 3
    };

private:		
    /** The buffers for all sound effects. These are shared among all
     *  instances of SFXOpenal. */
    std::vector<ALuint>       m_sfx_buffers;
    std::vector<int>          m_sfx_positional;
    std::vector<float>        m_sfx_rolloff;
    std::vector<float>        m_sfx_gain;
    std::vector<SFXBase*>     m_all_sfx;
    bool                      m_initialized;
    float                     m_masterGain;

    void                      loadSfx();

    void                      loadSingleSfx(const lisp::Lisp *lisp, 
                                            const  char *name, 
                                            int type);

public:
                             SFXManager();
    virtual                 ~SFXManager();
    bool                     sfxAllowed();
    int                       addSingleSfx( std::string    filename,
                                            int            positional,
                                            float          rolloff,
                                            float          gain);

    SFXBase                 *newSFX(int id);
    void                     deleteSFX(SFXBase *sfx);
    void                     pauseAll();
    void                     resumeAll();
    
    void                     setMasterSFXVolume(float gain);
    float                    getMasterSFXVolume() const { return m_masterGain; }
    
    static bool              checkError(const std::string &context);
    static const std::string getErrorString(int err);
};

extern SFXManager* sfx_manager;

#endif // HEADER_SFX_MANAGER_HPP

