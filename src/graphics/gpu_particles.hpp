//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2014-2015 SuperTuxKart-Team
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

#ifndef HEADER_GPU_PARTICLES_HPP
#define HEADER_GPU_PARTICLES_HPP

#include "graphics/shader.hpp"

#include "../lib/irrlicht/source/Irrlicht/CParticleSystemSceneNode.h"
#include <ISceneManager.h>
#include <IParticleSystemSceneNode.h>

namespace irr { namespace video{ class ITexture; } }

using namespace irr;

class ParticleSystemProxy : public scene::CParticleSystemSceneNode
{
protected:
    /** Transform feedback buffers: one to store the previous particle data,
     *  one to receice the updated data. Then the two buffers are swapped. */
    GLuint m_tfb_buffers[2];

    /** Stores the initial particle data. */
    GLuint m_initial_values_buffer;
    GLuint m_height_map_buffer;
    GLuint m_height_map_texture;
    GLuint m_quaternions_buffer;
    GLuint m_current_simulation_vao;
    GLuint m_non_current_simulation_vao;
    GLuint m_current_rendering_vao;
    GLuint m_non_current_rendering_vao;

    bool m_alpha_additive;
    bool m_has_height_map;
    bool m_flip;
    float size_increase_factor;
    float m_track_x, m_track_z;
    float m_track_x_len, m_track_z_len;
    float m_color_from[3];
    float m_color_to[3];
    bool m_first_execution;
    bool m_randomize_initial_y;

    GLuint m_texture;

    /** Current count of particles. */
    unsigned m_count;
    /** Previous count - for error handling only. */
    unsigned m_previous_count;

    static void setCommonRenderingVAO(GLuint position_buffer);
    static void appendQuaternionRenderingVAO(GLuint QuaternionBuffer);
    static void setCommonSimulationVAO(GLuint position_vbo,
                                       GLuint initialValues_vbo);

    void generateVAOs();
    void cleanGL();

    void drawFlip();
    void drawNotFlip();
    virtual void simulate();
    virtual void draw();

    struct ParticleData
    {
        float m_position_x;
        float m_position_y;
        float m_position_z;
        float m_lifetime;
        float m_direction_x;
        float m_direction_y;
        float m_direction_z;
        float m_size;
    };   // struct ParticleData

private:

    ParticleData *m_particle_params;
    ParticleData *m_initial_values;
    void generateParticlesFromPointEmitter(scene::IParticlePointEmitter *);
    void generateParticlesFromBoxEmitter(scene::IParticleBoxEmitter *);
    void generateParticlesFromSphereEmitter(scene::IParticleSphereEmitter *);
    void generateLifetimeSizeDirection(scene::IParticleEmitter *emitter,
                                       ParticleData *particle);
public:
    static IParticleSystemSceneNode *addParticleNode(
             bool withDefaultEmitter = true,
             bool randomize_initial_y = false,
             ISceneNode* parent = 0, s32 id = -1,
             const core::vector3df& position = core::vector3df(0, 0, 0),
             const core::vector3df& rotation = core::vector3df(0, 0, 0),
             const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));

    ParticleSystemProxy(bool createDefaultEmitter, ISceneNode* parent, 
                        scene::ISceneManager* mgr, s32 id,
                        const core::vector3df& position,
                        const core::vector3df& rotation,
                        const core::vector3df& scale,
                        bool randomize_initial_y);
    ~ParticleSystemProxy();

    virtual void setEmitter(scene::IParticleEmitter* emitter);
    virtual void render();
    void setHeightmap(const std::vector<std::vector<float> >&, float, float, 
                     float, float);

    // ------------------------------------------------------------------------
    void setFlip() { m_flip = true;}
    // ------------------------------------------------------------------------
    void setAlphaAdditive(bool val) { m_alpha_additive = val; }
    // ------------------------------------------------------------------------
    void setIncreaseFactor(float val) { size_increase_factor = val; }
    // ------------------------------------------------------------------------
    void setColorFrom(float r, float g, float b)
    {
        m_color_from[0] = r;
        m_color_from[1] = g;
        m_color_from[2] = b;
    }   // setColorFrom
    // ------------------------------------------------------------------------
    void setColorTo(float r, float g, float b)
    {
        m_color_to[0] = r;
        m_color_to[1] = g;
        m_color_to[2] = b;
    }   // setColorTo
    // ------------------------------------------------------------------------
    const float* getColorFrom() const { return m_color_from; }
    // ------------------------------------------------------------------------
    const float* getColorTo() const { return m_color_to; }
    // ------------------------------------------------------------------------
};   // ParticleSystemProxy

#endif // GPUPARTICLES_H
