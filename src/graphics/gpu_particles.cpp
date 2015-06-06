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

#include "graphics/gpu_particles.hpp"

#include "config/user_config.hpp"
#include "graphics/glwrap.hpp"
#include "graphics/irr_driver.hpp"
#include "graphics/particle_emitter.hpp"
#include "graphics/shaders.hpp"
#include "graphics/shared_gpu_objects.hpp"
#include "graphics/texture_shader.hpp"
#include "guiengine/engine.hpp"
#include "io/file_manager.hpp"

#include <ICameraSceneNode.h>
#include <IParticleSystemSceneNode.h>
#include "../../lib/irrlicht/source/Irrlicht/os.h"
#define COMPONENTCOUNT 8


// ============================================================================
/** Transform feedback shader that simulates the particles on GPU.
*/
class PointEmitterShader : public Shader
                           < PointEmitterShader, int, int, float >
{
public:
    PointEmitterShader()
    {
        const char *varyings[] = { "new_particle_position", "new_lifetime",
                                   "new_particle_velocity",  "new_size"     };
        loadTFBProgram("pointemitter.vert", varyings, 4);
        assignUniforms("dt", "level", "m_size_increase_factor");
    }   // PointEmitterShader
    // ------------------------------------------------------------------------
    void render(int timediff, int active_count, float size_increase_factor)
    {
        use();
        setUniforms(timediff, active_count, size_increase_factor);
    }   // render
};   // PointEmitterShader

// ============================================================================
/** A Shader to render particles.
*/
class SimpleParticleRender : public TextureShader<SimpleParticleRender, 2,
                                                  core::matrix4, video::SColorf,
                                                  video::SColorf >
{
public:
    SimpleParticleRender()
    {
        loadProgram(PARTICLES_RENDERING,
                    GL_VERTEX_SHADER,   "particle.vert",
                    GL_FRAGMENT_SHADER, "utils/getPosFromUVDepth.frag",
                    GL_FRAGMENT_SHADER, "particle.frag");

        assignUniforms("source_matrix", "color_from", "color_to");
        assignSamplerNames(0, "tex",  ST_TRILINEAR_ANISOTROPIC_FILTERED,
                           1, "dtex", ST_NEAREST_FILTERED);
    }   // SimpleParticleRender
    // ------------------------------------------------------------------------
    void render(bool alpha_additive,GLuint texture, const float *color_from, 
                const float *color_to, const core::matrix4 &transform,
                GLuint rendering_vao, int count)
    {
        if (alpha_additive)
            glBlendFunc(GL_ONE, GL_ONE);
        else
            glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        use();

        setTextureUnits(texture, irr_driver->getDepthStencilTexture());
        video::SColorf s_color_from = video::SColorf(color_from[0],
                                                     color_from[1],
                                                     color_from[2]);
        video::SColorf s_color_to = video::SColorf(color_to[0], color_to[1],
                                                   color_to[2]);

        setUniforms(transform, s_color_from, s_color_to);

        glBindVertexArray(rendering_vao);
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, count);
    }   // render

};   // SimpleParticleRender

// ============================================================================

class FlipParticleRender : public TextureShader<FlipParticleRender, 2>
{
public:
    FlipParticleRender()
    {
        loadProgram(PARTICLES_RENDERING,
                    GL_VERTEX_SHADER,   "flipparticle.vert",
                    GL_FRAGMENT_SHADER, "utils/getPosFromUVDepth.frag",
                    GL_FRAGMENT_SHADER, "particle.frag");
        assignUniforms();
        assignSamplerNames(0, "tex",  ST_TRILINEAR_ANISOTROPIC_FILTERED,
                           1, "dtex", ST_NEAREST_FILTERED);
    }   // FlipParticleRender

};   // FlipParticleShader

// ============================================================================
/** */
class HeightmapSimulationShader : public Shader <HeightmapSimulationShader,
                                                 core::matrix4, int, int,
                                                 float,float,float,float,float>
{
private:
    GLuint m_heightmap_texture_unit;
public:
    HeightmapSimulationShader()
    {
        const char *varyings[] = {"new_particle_position", "new_lifetime",
                                  "new_particle_velocity", "new_size"      };
        loadTFBProgram("particlesimheightmap.vert", varyings, 4);
        assignUniforms("sourcematrix", "dt", "level", "m_size_increase_factor",
                       "track_x", "track_x_len", "track_z", "track_z_len");
        m_heightmap_texture_unit = 2;
        assignTextureUnit(m_heightmap_texture_unit, "heightmap");
    }   // heightmapsimulationShader
    // ------------------------------------------------------------------------
    void render(const core::matrix4 &matrix, int timediff, int active_count,
                float m_size_increase_factor, float track_x, float track_x_len, 
                float track_z, float track_z_len, GLuint heightmap_texture)
    {
        use();
        glActiveTexture(GL_TEXTURE0 + m_heightmap_texture_unit);
        glBindTexture(GL_TEXTURE_BUFFER, heightmap_texture);
        setUniforms(matrix, timediff, active_count, m_size_increase_factor,
                    track_x, track_x_len, track_z, track_z_len);
    }   // render
};   // class HeightmapSimulationShader

// ============================================================================
/** Static function to create a particle proxy.
 */
scene::IParticleSystemSceneNode *ParticleSystemProxy::addParticleNode(
    bool withDefaultEmitter, bool randomize_initial_y, ISceneNode* parent, 
    s32 id, const core::vector3df& position, const core::vector3df& rotation,
    const core::vector3df& scale)
{
    if (!parent)
        parent = irr_driver->getSceneManager()->getRootSceneNode();

    IParticleSystemSceneNode* node = 
        new ParticleSystemProxy(withDefaultEmitter, parent,
                                irr_driver->getSceneManager(), id, position,
                                rotation, scale, randomize_initial_y);
    node->drop();

    return node;
}
// ============================================================================
ParticleSystemProxy::ParticleSystemProxy(bool createDefaultEmitter,
                                         ISceneNode* parent,
                                         scene::ISceneManager* mgr, s32 id,
                                         const core::vector3df& position,
                                         const core::vector3df& rotation,
                                         const core::vector3df& scale,
                                         bool randomize_initial_y)
                   : CParticleSystemSceneNode(createDefaultEmitter, parent,
                                              mgr, id, position, rotation, scale),
                     m_alpha_additive(false),
                     m_first_execution(true)
{
    if (randomize_initial_y)
        m_randomize_initial_y = randomize_initial_y;

    m_randomize_initial_y = randomize_initial_y;
    m_size_increase_factor = 0.;
    m_particle_params = NULL;
    m_initial_values = NULL;

    m_color_from[0] = m_color_from[1] = m_color_from[2] = 1.0;
    m_color_to[0] = m_color_to[1] = m_color_to[2] = 1.0;
    
    // We set these later but avoid coverity report them
    m_height_map_buffer = 0;
    m_height_map_texture = 0;
    m_has_height_map = false;
    m_flip = false;
    m_track_x = 0;
    m_track_z = 0;
    m_track_x_len = 0;
    m_track_z_len = 0;
    m_texture = 0;
}   // ParticleSystemProxy

// ----------------------------------------------------------------------------
ParticleSystemProxy::~ParticleSystemProxy()
{
    if (m_initial_values)
        free(m_initial_values);
    if (m_particle_params)
        free(m_particle_params);
    if (!m_first_execution)
        cleanGL();
    if (m_height_map_buffer)
        glDeleteBuffers(1, &m_height_map_buffer);
    if (m_height_map_texture)
        glDeleteTextures(1, &m_height_map_texture);
}   // ~ParticleSystemProxy

// ----------------------------------------------------------------------------
void ParticleSystemProxy::setHeightmap(const std::vector<std::vector<float> > &hm,
                                       float track_x, float track_z, 
                                       float track_x_len, float track_z_len)
{
    m_track_x     = track_x;
    m_track_z     = track_z;
    m_track_x_len = track_x_len;
    m_track_z_len = track_z_len;

    unsigned width  = (unsigned)hm.size();
    unsigned height = (unsigned)hm[0].size();
    float *hm_array = new float[width * height];
    for (unsigned i = 0; i < width; i++)
    {
        for (unsigned j = 0; j < height; j++)
        {
            hm_array[i * height + j] = hm[i][j];
        }
    }
    m_has_height_map = true;
    glGenBuffers(1, &m_height_map_buffer);
    glBindBuffer(GL_TEXTURE_BUFFER, m_height_map_buffer);
    glBufferData(GL_TEXTURE_BUFFER, width * height * sizeof(float), hm_array,
                 GL_STREAM_COPY);
    glGenTextures(1, &m_height_map_texture);
    glBindTexture(GL_TEXTURE_BUFFER, m_height_map_texture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, m_height_map_buffer);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);

    delete[] hm_array;
}   // setHeightmap

// ----------------------------------------------------------------------------
/** Generates the random initial data for a single particle.
 *  param emitter The emitter for this particle.
 *  \param particle Pointer to the particle data to be initialised.
 */
void ParticleSystemProxy::generateLifetimeSizeDirection
                                  (scene::IParticleEmitter *emitter,
                                   ParticleData *particle)
{
    float size_min = emitter->getMinStartSize().Height;
    float size_max = emitter->getMaxStartSize().Height;
    float lifetime_range = float(  emitter->getMaxLifeTime()
                                 - emitter->getMinLifeTime());

    particle->m_lifetime = os::Randomizer::frand() * lifetime_range
                         + emitter->getMinLifeTime();
    particle->m_size = os::Randomizer::frand();
    particle->m_size *= (size_max - size_min);
    particle->m_size += size_min;

    core::vector3df dir = emitter->getDirection();
    dir.rotateXYBy(os::Randomizer::frand() * emitter->getMaxAngleDegrees());
    dir.rotateYZBy(os::Randomizer::frand() * emitter->getMaxAngleDegrees());
    dir.rotateXZBy(os::Randomizer::frand() * emitter->getMaxAngleDegrees());

    particle->m_direction_x = dir.X;
    particle->m_direction_y = dir.Y;
    particle->m_direction_z = dir.Z;
}   // generateLifetimeSizeDirection

// ----------------------------------------------------------------------------
void ParticleSystemProxy::generateParticlesFromPointEmitter
                          (scene::IParticlePointEmitter *emitter)
{
    ParticleData* particle_params_tmp =
        (ParticleData *) realloc(m_particle_params,
                                 sizeof(ParticleData) * m_count);
    ParticleData* initial_values_tmp = 
        (ParticleData *)realloc(m_initial_values,
                                sizeof(ParticleData)* m_count);

    if (particle_params_tmp)     // In case memory allocation succeeded
    {
        m_particle_params = particle_params_tmp;
    }
    else
    {
        Log::error("GPUParticles", 
                   "Not enough memory for %d from point particles.", m_count);
        m_count = m_previous_count;
    }
    if (initial_values_tmp)
    {
        m_initial_values = initial_values_tmp;
    }
    else
    {
        Log::fatal("GPUParticles",
                   "Not enough memory for %d from point particles.", m_count);
        m_count = m_previous_count;
    }

    for (unsigned i = 0; i < m_count; i++)
    {
        m_particle_params[i].m_position_x = 0;
        m_particle_params[i].m_position_y = 0;
        m_particle_params[i].m_position_z = 0;
        // Initial lifetime is >1
        m_initial_values[i].m_lifetime = 2.;

        memcpy(&(m_initial_values[i].m_position_x),
               &(m_particle_params[i].m_position_x),
               3 * sizeof(float));

        generateLifetimeSizeDirection(emitter, &(m_particle_params[i]));

        memcpy(&(m_initial_values[i].m_direction_x),
               &(m_particle_params[i].m_direction_x), 4 * sizeof(float));
    }
}   // generateParticlesFromPointEmitter

// ----------------------------------------------------------------------------
void ParticleSystemProxy::generateParticlesFromBoxEmitter
                                          (scene::IParticleBoxEmitter *emitter)
{
    ParticleData* particle_params_tmp =
        (ParticleData *) realloc(m_particle_params, 
                                 sizeof(ParticleData) * m_count);
    ParticleData* initial_values_tmp =
        (ParticleData *)realloc(m_initial_values,
                                sizeof(ParticleData)* m_count);

    if (particle_params_tmp)     // In case memory allocation succeeded
    {
        m_particle_params = particle_params_tmp;
    }
    else
    {
        Log::error("GPUParticles",
                   "Not enough memory for %d from box particles.", m_count);
        m_count = m_previous_count;
    }
    if (initial_values_tmp)
    {
        m_initial_values = initial_values_tmp;
    }
    else
    {
        Log::error("GPUParticles",
                   "Not enough memory for %d from box particles.", m_count);
        m_count = m_previous_count;
    }

    const core::vector3df& extent = emitter->getBox().getExtent();

    for (unsigned i = 0; i < m_count; i++)
    {
        m_particle_params[i].m_position_x = 
            emitter->getBox().MinEdge.X + os::Randomizer::frand() * extent.X;
        m_particle_params[i].m_position_y =
            emitter->getBox().MinEdge.Y + os::Randomizer::frand() * extent.Y;
        m_particle_params[i].m_position_z =
            emitter->getBox().MinEdge.Z + os::Randomizer::frand() * extent.Z;
        // Initial lifetime is random
        m_initial_values[i].m_lifetime = os::Randomizer::frand();
        if (!m_randomize_initial_y)
            m_initial_values[i].m_lifetime += 1.;

        memcpy(&(m_initial_values[i].m_position_x),
               &(m_particle_params[i].m_position_x), 3 * sizeof(float));
        generateLifetimeSizeDirection(emitter, &(m_particle_params[i]));
        memcpy(&(m_initial_values[i].m_direction_x),
               &(m_particle_params[i].m_direction_x), 4 * sizeof(float));

        if (m_randomize_initial_y)
            m_initial_values[i].m_position_y = os::Randomizer::frand()*50.0f;
    }
}   // generateParticlesFromBoxEmitter

// ----------------------------------------------------------------------------
void ParticleSystemProxy::generateParticlesFromSphereEmitter
                                       (scene::IParticleSphereEmitter *emitter)
{
    ParticleData* particle_params_tmp =
        (ParticleData *) realloc(m_particle_params,
                                 sizeof(ParticleData) * m_count);
    ParticleData* initial_values_tmp =
        (ParticleData *)realloc(m_initial_values, sizeof(ParticleData)* m_count);

    if(particle_params_tmp != NULL)     // In case memory allocation succeeded
        m_particle_params = particle_params_tmp;
    if(initial_values_tmp != NULL)
        m_initial_values = initial_values_tmp;

    for (unsigned i = 0; i < m_count; i++) {
        // Random distance from center
        const f32 distance = os::Randomizer::frand() * emitter->getRadius();

        // Random direction from center
        vector3df pos = emitter->getCenter() + distance;
        pos.rotateXYBy(os::Randomizer::frand() * 360.f, emitter->getCenter());
        pos.rotateYZBy(os::Randomizer::frand() * 360.f, emitter->getCenter());
        pos.rotateXZBy(os::Randomizer::frand() * 360.f, emitter->getCenter());

        m_particle_params[i].m_position_x = pos.X;
        m_particle_params[i].m_position_y = pos.Y;
        m_particle_params[i].m_position_z = pos.Z;
        // Initial lifetime is > 1
        m_initial_values[i].m_lifetime = 2.;

        memcpy(&(m_initial_values[i].m_position_x),
               &(m_particle_params[i].m_position_x), 3 * sizeof(float));
        generateLifetimeSizeDirection(emitter, &(m_particle_params[i]));
        memcpy(&(m_initial_values[i].m_direction_x),
               &(m_particle_params[i].m_direction_x), 4 * sizeof(float));
    }
}   // generateParticlesFromSphereEmitter

// ----------------------------------------------------------------------------
/** Returns if the specified particle type can be done on HPU.
 *  \param type Type to check.
 */
static bool isGPUParticleType(scene::E_PARTICLE_EMITTER_TYPE type)
{
    switch (type)
    {
    case scene::EPET_POINT:
    case scene::EPET_BOX:
    case scene::EPET_SPHERE:
        return true;
    default:
        return false;
    }
}   // isGPUParticleType

// ----------------------------------------------------------------------------
void ParticleSystemProxy::setEmitter(scene::IParticleEmitter* emitter)
{
    CParticleSystemSceneNode::setEmitter(emitter);
    if (!emitter || !isGPUParticleType(emitter->getType()))
        return;
    if (!m_first_execution)
        cleanGL();
    m_has_height_map = false;
    m_flip = false;
    m_first_execution = true;

    m_previous_count = m_count;   // save to handle out of memory errors
    m_count = emitter->getMaxParticlesPerSecond() * emitter->getMaxLifeTime()
                                                  / 1000;
    switch (emitter->getType())
    {
    case scene::EPET_POINT:
        generateParticlesFromPointEmitter(emitter);
        break;
    case scene::EPET_BOX:
        generateParticlesFromBoxEmitter(
                           static_cast<scene::IParticleBoxEmitter *>(emitter));
        break;
    case scene::EPET_SPHERE:
        generateParticlesFromSphereEmitter(
                        static_cast<scene::IParticleSphereEmitter *>(emitter));
        break;
    default:
        assert(0 && "Wrong particle type");
    }

    video::ITexture *tex = getMaterial(0).getTexture(0);
    compressTexture(tex, true, true);
    m_texture = getTextureGLuint(getMaterial(0).getTexture(0));
}

// ----------------------------------------------------------------------------
void ParticleSystemProxy::cleanGL()
{
    if (m_flip)
        glDeleteBuffers(1, &m_quaternions_buffer);
    glDeleteBuffers(2, m_tfb_buffers);
    glDeleteBuffers(1, &m_initial_values_buffer);
    glDeleteVertexArrays(1, &m_current_rendering_vao);
    glDeleteVertexArrays(1, &m_non_current_rendering_vao);
    glDeleteVertexArrays(1, &m_current_simulation_vao);
    glDeleteVertexArrays(1, &m_non_current_simulation_vao);
}   // cleanGL

// ----------------------------------------------------------------------------
void ParticleSystemProxy::setCommonRenderingVAO(GLuint position_buffer)
{
    glBindBuffer(GL_ARRAY_BUFFER, SharedGPUObjects::getParticleQuadVBO());
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (GLvoid *)(2 * sizeof(float)));

    glBindBuffer(GL_ARRAY_BUFFER, position_buffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleData), 0);
    glVertexAttribDivisorARB(0, 1);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleData),
                          (GLvoid *)(3 * sizeof(float)));
    glVertexAttribDivisorARB(1, 1);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleData),
                          (GLvoid *)(7 * sizeof(float)));
    glVertexAttribDivisorARB(2, 1);
}   // setCommonRenderingVAO

// ----------------------------------------------------------------------------
void ParticleSystemProxy::appendQuaternionRenderingVAO(GLuint quaternion_buffer)
{
    glBindBuffer(GL_ARRAY_BUFFER, quaternion_buffer);
    glEnableVertexAttribArray(5);

    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glVertexAttribDivisorARB(5, 1);

    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (GLvoid *)(3 * sizeof(float)));
    glVertexAttribDivisorARB(6, 1);
}   // appendQuaternionRenderingVAO

// ----------------------------------------------------------------------------
void ParticleSystemProxy::setCommonSimulationVAO(GLuint position_vbo,
                                                GLuint initialValues_vbo)
{
    glBindBuffer(GL_ARRAY_BUFFER, position_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(4 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(7 * sizeof(float)));

    glBindBuffer(GL_ARRAY_BUFFER, initialValues_vbo);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)0);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(3 * sizeof(float)));
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(4 * sizeof(float)));
    glEnableVertexAttribArray(7);
    glVertexAttribPointer(7, 1, GL_FLOAT, GL_FALSE,
                          sizeof(ParticleSystemProxy::ParticleData),
                          (GLvoid*)(7 * sizeof(float)));
}   // setCommonSimulationVAO

// ----------------------------------------------------------------------------
void ParticleSystemProxy::simulate()
{
    int timediff = int(GUIEngine::getLatestDt() * 1000.f);
    int active_count = getEmitter()->getMaxLifeTime() 
                     * getEmitter()->getMaxParticlesPerSecond() / 1000;
    core::matrix4 matrix = getAbsoluteTransformation();

    glEnable(GL_RASTERIZER_DISCARD);
    if (m_has_height_map)
    {
        HeightmapSimulationShader::getInstance()
            ->render(matrix, timediff, active_count, m_size_increase_factor,
                     m_track_x, m_track_x_len, m_track_z, m_track_z_len,
                     m_height_map_texture);
    }
    else
    {
        PointEmitterShader::getInstance()->render(timediff, active_count,
                                                  m_size_increase_factor);
    }

    glBindVertexArray(m_current_simulation_vao);
    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, m_tfb_buffers[1]);

    glBeginTransformFeedback(GL_POINTS);
    glDrawArrays(GL_POINTS, 0, m_count);
    glEndTransformFeedback();
    glBindVertexArray(0);

    glDisable(GL_RASTERIZER_DISCARD);
#define DEBUG_PARTICLES
#ifdef DEBUG_PARTICLES
    // This code maps the data from the particles (initial data, current data
    // and output of transform feedback shader) into memory. Useful for debugging.
    
    //if (std::string(getName()) == std::string("particles(C:/Users/jhenrich/"
    //                 "supertuxkart/stk-assets/textures/skid-particle1.png)"))
    if (std::string(getName()) ==
        std::string("particles(C:/Users/jhenrich/supertuxkart/stk-assets/"
                    "textures/nitro-particle.png)"))
    {
        glBindVertexArray(m_current_simulation_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_initial_values_buffer);
        ParticleData *p_initial =
            (ParticleData*)glMapBufferRange(GL_ARRAY_BUFFER, 0,
                                            m_count*sizeof(ParticleData),
                                            GL_MAP_READ_BIT);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, m_tfb_buffers[0]);
        ParticleData *p_prev =
            (ParticleData*)glMapBufferRange(GL_ARRAY_BUFFER, 0,
                                            m_count*sizeof(ParticleData),
                                            GL_MAP_READ_BIT);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, m_tfb_buffers[1]);
        ParticleData *p_new =
            (ParticleData*)glMapBufferRange(GL_ARRAY_BUFFER, 0,
                                            m_count*sizeof(ParticleData),
                                            GL_MAP_READ_BIT);
        glUnmapBuffer(GL_ARRAY_BUFFER);
    }
#endif

    std::swap(m_tfb_buffers[0], m_tfb_buffers[1]);
    std::swap(m_current_rendering_vao, m_non_current_rendering_vao);
    std::swap(m_current_simulation_vao, m_non_current_simulation_vao);
}   // simulate

// ----------------------------------------------------------------------------
void ParticleSystemProxy::drawFlip()
{
    glBlendFunc(GL_ONE, GL_ONE);
    FlipParticleRender::getInstance()->use();

    FlipParticleRender::getInstance()
        ->setTextureUnits(m_texture, irr_driver->getDepthStencilTexture());
    FlipParticleRender::getInstance()->setUniforms();

    glBindVertexArray(m_current_rendering_vao);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, m_count);
}   // drawFlip

// ----------------------------------------------------------------------------
void ParticleSystemProxy::drawNotFlip()
{
    SimpleParticleRender::getInstance()->render(m_alpha_additive, m_texture,
                                                getColorFrom(), getColorTo(),
                                                getAbsoluteTransformation(),
                                                m_current_rendering_vao, m_count
        );
    glBindVertexArray(m_current_rendering_vao);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, m_count);
}   // drawNotFlip

// ----------------------------------------------------------------------------
void ParticleSystemProxy::draw()
{
    if (m_flip)
        drawFlip();
    else
        drawNotFlip();
}   // draw

// ----------------------------------------------------------------------------
void ParticleSystemProxy::generateVAOs()
{
    glBindVertexArray(0);
    glGenBuffers(1, &m_initial_values_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_initial_values_buffer);
    glBufferData(GL_ARRAY_BUFFER, m_count * sizeof(ParticleData),
                  m_particle_params, GL_STREAM_COPY);
    glGenBuffers(2, m_tfb_buffers);
    glBindBuffer(GL_ARRAY_BUFFER, m_tfb_buffers[0]);
    glBufferData(GL_ARRAY_BUFFER, m_count * sizeof(ParticleData),
                 m_initial_values, GL_STREAM_COPY);
    glBindBuffer(GL_ARRAY_BUFFER, m_tfb_buffers[1]);
    glBufferData(GL_ARRAY_BUFFER, m_count * sizeof(ParticleData), 0,
                 GL_STREAM_COPY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenVertexArrays(1, &m_current_rendering_vao);
    glGenVertexArrays(1, &m_non_current_rendering_vao);
    glGenVertexArrays(1, &m_current_simulation_vao);
    glGenVertexArrays(1, &m_non_current_simulation_vao);

    glBindVertexArray(m_current_simulation_vao);
    setCommonSimulationVAO(m_tfb_buffers[0], m_initial_values_buffer);
    glBindVertexArray(m_non_current_simulation_vao);
    setCommonSimulationVAO(m_tfb_buffers[1], m_initial_values_buffer);


    glBindVertexArray(0);
    if (m_flip)
    {
        float *quaternions = new float[4 * m_count];
        for (unsigned i = 0; i < m_count; i++)
        {
            core::vector3df rotationdir(0., 1., 0.);

            quaternions[4 * i] = rotationdir.X;
            quaternions[4 * i + 1] = rotationdir.Y;
            quaternions[4 * i + 2] = rotationdir.Z;
             // 3 half rotation during lifetime at max
            quaternions[4 * i + 3] = 3.14f * 3.f 
                                   * (2.f * os::Randomizer::frand() - 1.f);
        }
        glGenBuffers(1, &m_quaternions_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_quaternions_buffer);
        glBufferData(GL_ARRAY_BUFFER, 4 * m_count * sizeof(float),
                     quaternions, GL_STREAM_COPY);
        delete[] quaternions;
    }

    glBindVertexArray(m_current_rendering_vao);
    setCommonRenderingVAO(m_tfb_buffers[0]);
    if (m_flip)
        appendQuaternionRenderingVAO(m_quaternions_buffer);

    glBindVertexArray(m_non_current_rendering_vao);
    setCommonRenderingVAO(m_tfb_buffers[1]);
    if (m_flip)
        appendQuaternionRenderingVAO(m_quaternions_buffer);
    glBindVertexArray(0);
}   // generateVAOs

// ----------------------------------------------------------------------------
void ParticleSystemProxy::render()
{
    if (!getEmitter() || !isGPUParticleType(getEmitter()->getType()))
    {
        CParticleSystemSceneNode::render();
        return;
    }
    if (m_first_execution)
        generateVAOs();
    m_first_execution = false;
    simulate();
    draw();
}   // render
