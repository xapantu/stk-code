//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2006-2015 Joerg Henrichs
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

#include "physics/physics.hpp"

#include "achievements/achievement_info.hpp"
#include "animations/three_d_animation.hpp"
#include "config/player_manager.hpp"
#include "config/player_profile.hpp"
#include "config/user_config.hpp"
#include "karts/abstract_kart.hpp"
#include "graphics/irr_driver.hpp"
#include "graphics/stars.hpp"
#include "items/flyable.hpp"
#include "karts/kart_properties.hpp"
#include "karts/rescue_animation.hpp"
#include "karts/controller/player_controller.hpp"
#include "modes/soccer_world.hpp"
#include "modes/world.hpp"
#include "karts/explosion_animation.hpp"
#include "physics/btKart.hpp"
#include "physics/irr_debug_drawer.hpp"
#include "physics/physical_object.hpp"
#include "physics/stk_dynamics_world.hpp"
#include "physics/triangle_mesh.hpp"
#include "race/race_manager.hpp"
#include "scriptengine/script_engine.hpp"
#include "tracks/track.hpp"
#include "tracks/track_object.hpp"
#include "utils/profiler.hpp"

// ----------------------------------------------------------------------------
/** Initialise physics.
 *  Create the bullet dynamics world.
 */
Physics::Physics() : btSequentialImpulseConstraintSolver()
{
    m_collision_conf      = new btDefaultCollisionConfiguration();
    m_dispatcher          = new btCollisionDispatcher(m_collision_conf);
}   // Physics

//-----------------------------------------------------------------------------
/** The actual initialisation of the physics, which is called after the track
 *  model is loaded. This allows the physics to use the actual track dimension
 *  for the axis sweep.
 */
void Physics::init(const Vec3 &world_min, const Vec3 &world_max)
{
    m_physics_loop_active = false;
    m_axis_sweep          = new btAxisSweep3(world_min, world_max);
    m_dynamics_world      = new STKDynamicsWorld(m_dispatcher,
                                                 m_axis_sweep,
                                                 this,
                                                 m_collision_conf);
    m_karts_to_delete.clear();
    m_dynamics_world->setGravity(
        btVector3(0.0f,
                  -World::getWorld()->getTrack()->getGravity(),
                  0.0f));
    m_debug_drawer = new IrrDebugDrawer();
    m_dynamics_world->setDebugDrawer(m_debug_drawer);
}   // init

//-----------------------------------------------------------------------------
Physics::~Physics()
{
    delete m_debug_drawer;
    delete m_dynamics_world;
    delete m_axis_sweep;
    delete m_dispatcher;
    delete m_collision_conf;
}   // ~Physics

// ----------------------------------------------------------------------------
/** Adds a kart to the physics engine.
 *  This adds the rigid body and the vehicle but only if the kart is not
 *  already in the physics world.
 *  \param kart The kart to add.
 *  \param vehicle The raycast vehicle object.
 */
void Physics::addKart(const AbstractKart *kart)
{
    const btCollisionObjectArray &all_objs =
        m_dynamics_world->getCollisionObjectArray();
    for(unsigned int i=0; i<(unsigned int)all_objs.size(); i++)
    {
        if(btRigidBody::upcast(all_objs[i])== kart->getBody())
            return;
    }
    m_dynamics_world->addRigidBody(kart->getBody());
    m_dynamics_world->addVehicle(kart->getVehicle());
}   // addKart

//-----------------------------------------------------------------------------
/** Removes a kart from the physics engine. This is used when rescuing a kart
 *  (and during cleanup).
 *  \param kart The kart to remove.
 */
void Physics::removeKart(const AbstractKart *kart)
{
    // We can't simply remove a kart from the physics world when currently
    // loops over all kart objects are active. This can happen in collision
    // handling, where a collision of a kart with a cake etc. removes
    // a kart from the physics. In this case save pointers to the kart
    // to be removed, and remove them once the physics processing is done.
    if(m_physics_loop_active)
    {
        // Make sure to remove each kart only once.
        if(std::find(m_karts_to_delete.begin(), m_karts_to_delete.end(), kart)
                     == m_karts_to_delete.end())
        {
            m_karts_to_delete.push_back(kart);
        }
    }
    else
    {
        m_dynamics_world->removeRigidBody(kart->getBody());
        m_dynamics_world->removeVehicle(kart->getVehicle());
    }
}   // removeKart

//-----------------------------------------------------------------------------
/** Updates the physics simulation and handles all collisions.
 *  \param dt Time step.
 */
void Physics::update(float dt)
{
    PROFILER_PUSH_CPU_MARKER("Physics", 0, 0, 0);

    m_physics_loop_active = true;
    // Bullet can report the same collision more than once (up to 4
    // contact points per collision). Additionally, more than one internal
    // substep might be taken, resulting in potentially even more
    // duplicates. To handle this, all collisions (i.e. pair of objects)
    // are stored in a vector, but only one entry per collision pair
    // of objects.
    m_all_collisions.clear();

    // Maximum of three substeps. This will work for framerate down to
    // 20 FPS (bullet default frequency is 60 HZ).
    m_dynamics_world->stepSimulation(dt, 3);

    // Now handle the actual collision. Note: flyables can not be removed
    // inside of this loop, since the same flyables might hit more than one
    // other object. So only a flag is set in the flyables, the actual
    // clean up is then done later in the projectile manager.
    std::vector<CollisionPair>::iterator p;
    for(p=m_all_collisions.begin(); p!=m_all_collisions.end(); ++p)
    {
        // Kart-kart collision
        // --------------------
        if(p->getUserPointer(0)->is(UserPointer::UP_KART))
        {
            KartKartCollision(p->getUserPointer(0)->getPointerKart(),
                              p->getContactPointCS(0),
                              p->getUserPointer(1)->getPointerKart(),
                              p->getContactPointCS(1)                );
            Scripting::ScriptEngine* script_engine = World::getWorld()->getScriptEngine();
            int kartid1 = p->getUserPointer(0)->getPointerKart()->getWorldKartId();
            int kartid2 = p->getUserPointer(1)->getPointerKart()->getWorldKartId();
            script_engine->runFunction(false, "void onKartKartCollision(int, int)",
                [=](asIScriptContext* ctx) {
                    ctx->SetArgDWord(0, kartid1);
                    ctx->SetArgDWord(1, kartid2);
                });
            continue;
        }  // if kart-kart collision

        if(p->getUserPointer(0)->is(UserPointer::UP_PHYSICAL_OBJECT))
        {
            // Kart hits physical object
            // -------------------------
            Scripting::ScriptEngine* script_engine = World::getWorld()->getScriptEngine();
            AbstractKart *kart = p->getUserPointer(1)->getPointerKart();
            int kartId = kart->getWorldKartId();
            PhysicalObject* obj = p->getUserPointer(0)->getPointerPhysicalObject();
            std::string obj_id = obj->getID();
            std::string scripting_function = obj->getOnKartCollisionFunction();

            TrackObject* to = obj->getTrackObject();
            TrackObject* library = to->getParentLibrary();
            std::string lib_id;
            std::string* lib_id_ptr = NULL;
            if (library != NULL)
                lib_id = library->getID();
            lib_id_ptr = &lib_id;

            if (scripting_function.size() > 0)
            {
                script_engine->runFunction(true, "void " + scripting_function + "(int, const string, const string)",
                    [&](asIScriptContext* ctx) {
                        ctx->SetArgDWord(0, kartId);
                        ctx->SetArgObject(1, lib_id_ptr);
                        ctx->SetArgObject(2, &obj_id);
                    });
            }
            if (obj->isCrashReset())
            {
                new RescueAnimation(kart);
            }
            else if (obj->isExplodeKartObject())
            {
                ExplosionAnimation::create(kart);
            }
            else if (obj->isFlattenKartObject())
            {
                const KartProperties* kp = kart->getKartProperties();
                kart->setSquash(kp->getSquashDuration() * kart->getPlayerDifficulty()->getSquashDuration(),
                    kp->getSquashSlowdown() * kart->getPlayerDifficulty()->getSquashSlowdown());
            }
            else if(obj->isSoccerBall() && 
                    race_manager->getMinorMode() == RaceManager::MINOR_MODE_SOCCER)
            {
                SoccerWorld* soccerWorld = (SoccerWorld*)World::getWorld();
                soccerWorld->setLastKartTohitBall(kartId);
            }
            continue;
        }

        if (p->getUserPointer(0)->is(UserPointer::UP_ANIMATION))
        {
            // Kart hits animation
            ThreeDAnimation *anim=p->getUserPointer(0)->getPointerAnimation();
            if(anim->isCrashReset())
            {
                AbstractKart *kart = p->getUserPointer(1)->getPointerKart();
                new RescueAnimation(kart);
            }
            else if (anim->isExplodeKartObject())
            {
                AbstractKart *kart = p->getUserPointer(1)->getPointerKart();
                ExplosionAnimation::create(kart);
            }
            else if (anim->isFlattenKartObject())
            {
                AbstractKart *kart = p->getUserPointer(1)->getPointerKart();
                const KartProperties* kp = kart->getKartProperties();
                kart->setSquash(kp->getSquashDuration() * kart->getPlayerDifficulty()->getSquashDuration(),
                    kp->getSquashSlowdown() * kart->getPlayerDifficulty()->getSquashSlowdown());
            }
            continue;

        }
        // now the first object must be a projectile
        // =========================================
        if(p->getUserPointer(1)->is(UserPointer::UP_TRACK))
        {
            // Projectile hits track
            // ---------------------
            p->getUserPointer(0)->getPointerFlyable()->hitTrack();
        }
        else if(p->getUserPointer(1)->is(UserPointer::UP_PHYSICAL_OBJECT))
        {
            // Projectile hits physical object
            // -------------------------------
            Scripting::ScriptEngine* script_engine = World::getWorld()->getScriptEngine();
            Flyable* flyable = p->getUserPointer(0)->getPointerFlyable();
            PhysicalObject* obj = p->getUserPointer(1)->getPointerPhysicalObject();
            std::string obj_id = obj->getID();
            std::string scripting_function = obj->getOnItemCollisionFunction();
            if (scripting_function.size() > 0)
            {
                script_engine->runFunction(true, "void " + scripting_function + "(int, int, const string)",
                        [&](asIScriptContext* ctx) {
                        ctx->SetArgDWord(0, (int)flyable->getType());
                        ctx->SetArgDWord(1, flyable->getOwnerId());
                        ctx->SetArgObject(2, &obj_id);
                    });
            }
            flyable->hit(NULL, obj);

            if (obj->isSoccerBall() && 
                race_manager->getMinorMode() == RaceManager::MINOR_MODE_SOCCER)
            {
                int kartId = p->getUserPointer(0)->getPointerFlyable()->getOwnerId();
                SoccerWorld* soccerWorld = (SoccerWorld*)World::getWorld();
                soccerWorld->setLastKartTohitBall(kartId);
            }

        }
        else if(p->getUserPointer(1)->is(UserPointer::UP_KART))
        {
            // Projectile hits kart
            // --------------------
            // Only explode a bowling ball if the target is
            // not invulnerable
            AbstractKart* target_kart = p->getUserPointer(1)->getPointerKart();
            PowerupManager::PowerupType type = p->getUserPointer(0)->getPointerFlyable()->getType();
            if(type != PowerupManager::POWERUP_BOWLING || !target_kart->isInvulnerable())
            {
                Flyable *f = p->getUserPointer(0)->getPointerFlyable();
                f->hit(target_kart);

                // Check for achievements
                AbstractKart * kart = World::getWorld()->getKart(f->getOwnerId());
                PlayerController *c = dynamic_cast<PlayerController*>(kart->getController());

                // Check that it's not a kart hitting itself (this can
                // happen at the time a flyable is shot - release too close
                // to the kart, and it's the current player. At this stage
                // only the current player can get achievements.
                if (target_kart != kart && c &&
                    c->getPlayer()->getConstProfile() == PlayerManager::getCurrentPlayer())
                {
                    // Compare the current value of hits with the 'hit' goal value
                    // (otherwise it would be compared with the kart id goal value,
                    // which doesn't exist.
                    PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_ARCH_ENEMY,
                                                       target_kart->getIdent(), 1, "hit");
                    if (type == PowerupManager::POWERUP_BOWLING)
                    {
                        PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_STRIKE,
                                                          "ball", 1);
                    }   // is bowling ball
                }   // if target_kart != kart && is a player kart and is current player
            }

        }
        else
        {
            // Projectile hits projectile
            // --------------------------
            p->getUserPointer(0)->getPointerFlyable()->hit(NULL);
            p->getUserPointer(1)->getPointerFlyable()->hit(NULL);
        }
    }  // for all p in m_all_collisions

    m_physics_loop_active = false;
    // Now remove the karts that were removed while the above loop
    // was active. Now we can safely call removeKart, since the loop
    // is finished and m_physics_world_active is not set anymore.
    for(unsigned int i=0; i<m_karts_to_delete.size(); i++)
        removeKart(m_karts_to_delete[i]);
    m_karts_to_delete.clear();

    PROFILER_POP_CPU_MARKER();
}   // update

//-----------------------------------------------------------------------------
/** Handles the special case of two karts colliding with each other, which
 *  means that bombs must be passed on. If both karts have a bomb, they'll
 *  explode immediately. This function is called from physics::update() on the
 *  server and if no networking is used, and from race_state on the client to
 *  replay what happened on the server.
 *  \param kart_a First kart involved in the collision.
 *  \param contact_point_a Location of collision at first kart (in kart
 *         coordinates).
 *  \param kart_b Second kart involved in the collision.
 *  \param contact_point_b Location of collision at second kart (in kart
 *         coordinates).
 */
void Physics::KartKartCollision(AbstractKart *kart_a,
                                const Vec3 &contact_point_a,
                                AbstractKart *kart_b,
                                const Vec3 &contact_point_b)
{
    // Only one kart needs to handle the attachments, it will
    // fix the attachments for the other kart.
    kart_a->crashed(kart_b, /*handle_attachments*/true);
    kart_b->crashed(kart_a, /*handle_attachments*/false);

    AbstractKart *left_kart, *right_kart;

    // Determine which kart is pushed to the left, and which one to the
    // right. Ideally the sign of the X coordinate of the local conact point
    // could decide the direction (negative X --> was hit on left side, gets
    // push to right), but that can lead to both karts being pushed in the
    // same direction (front left of kart hits rear left).
    // So we just use a simple test (which does the right thing in ideal
    // crashes, but avoids pushing both karts in corner cases
    // - pun intended ;) ).
    if(contact_point_a.getX() < contact_point_b.getX())
    {
        left_kart  = kart_b;
        right_kart = kart_a;
    }
    else
    {
        left_kart  = kart_a;
        right_kart = kart_b;
    }

    // The two karts will push each other away. To increase this effect,
    // add an impulse based on the speed (i.e. if a kart is actively trying
    // to push the other kart away, it will have a higher impulse) and weight
    // of the karts

    // 1. Check if a kart is more 'actively' trying to push another kart
    // -----------------------------------------------------------------
    // by checking its local sidewards velocity. Keep track of this in
    // a speed based factor: it indicates how much more the right kart
    // pushes the left kart. 
    float f_speed = 1.0f;

    // Project the speed of the karts onto a line connecting the two karts
    // to see which kart is more actively trying to ram the other kart
    Vec3 line = right_kart->getXYZ() - left_kart->getXYZ();

    // The speed factor depends on the ratio of the speeds projected on
    // 'line'.  Projection of vector v on u is ( (v.u) = dot product):
    // (v.u)/len(u)^2 * u 
    // Then the length of that is:
    // (v.u)/len(u)^2 * len(u) = (v.u)/len(u)
    // Since we only use the ratio, the division by
    // len(u) is not necessary.
    Vec3 v1 = left_kart->getPreviousVelocity();
    Vec3 v2 = left_kart->getVelocity();
    float fv1 = fabsf(line.dot(v1));
    float fv2 = fabsf(line.dot(v2));
    Vec3 w1 = right_kart->getPreviousVelocity();
    Vec3 w2 = right_kart->getVelocity();
    float fw1 = fabsf(line.dot(w1));
    float fw2 = fabsf(line.dot(w2));

    float left_speed_factor  = fabsf( line.dot(left_kart ->getVelocity() ) );
    float right_speed_factor = fabsf( line.dot(right_kart->getVelocity() ) );
    left_speed_factor = fabsf(left_kart->getVelocityLC().getX());
    right_speed_factor = fabsf(right_kart->getVelocityLC().getX());
    if (UserConfigParams::m_physics_debug)
    {
        Log::verbose("push", "line %f %f %f left speed %f %f %f f %f right speed %f %f %f factor %f",
            line.getX(), line.getY(), line.getZ(),
            left_kart->getVelocity().getX(), left_kart->getVelocity().getY(), left_kart->getVelocity().getZ(),
            left_speed_factor,
            right_kart->getVelocity().getX(), right_kart->getVelocity().getY(), right_kart->getVelocity().getZ(),
            right_speed_factor);

    }

    // Cap of the speed factor, to avoid pushing karts too much
    const float MAX_SPEED_FACTOR = 3.0f;
    if (left_speed_factor > 0)
    {
        f_speed = right_speed_factor / left_speed_factor;
        btClamp(f_speed, 1 / MAX_SPEED_FACTOR, MAX_SPEED_FACTOR);
    }
    else
        f_speed = MAX_SPEED_FACTOR;


    // 2. Compute a scaling factor dependent on the weight of the karts
    // -----------------------------------------------------------------

    // Add a scaling factor depending on the mass (avoid div by zero).
    // The value of f_right is applied to the right kart, and f_left
    // to the left kart. f_left = 1 / f_right
    float f_weight = right_kart->getKartProperties()->getMass()
                   / left_kart->getKartProperties()->getMass();
    const float MAX_WEIGHT_FACTOR = 2.0f;

    btClamp(f_weight, 1 / MAX_WEIGHT_FACTOR, MAX_WEIGHT_FACTOR);


    float f = f_weight * f_speed;

    if (f < 0.5)
        f = 0;
    else if (f>2)
        f = 6;

    // First push one kart to the left (if there is not already
    // an impulse happening - one collision might cause more
    // than one impulse otherwise)
    if(right_kart->getVehicle()->getCentralImpulseTime()<=0 && f>0)
    {
        const KartProperties *kp = left_kart->getKartProperties();
        Vec3 impulse(kp->getCollisionImpulse()/f, 0, 0);
        impulse = right_kart->getTrans().getBasis() * impulse;
        right_kart->getVehicle()
                 ->setTimedCentralImpulse(kp->getCollisionImpulseTime(),
                                          impulse);
        right_kart ->getBody()->setAngularVelocity(btVector3(0,0,0));
    }

    // Then push the other kart to the right (if there is no
    // impulse happening atm).
    if(left_kart->getVehicle()->getCentralImpulseTime()<=0)
    {
        const KartProperties *kp = right_kart->getKartProperties();
        Vec3 impulse = Vec3(-kp->getCollisionImpulse()*f, 0, 0);
        impulse = left_kart->getTrans().getBasis() * impulse;
        left_kart->getVehicle()
                  ->setTimedCentralImpulse(kp->getCollisionImpulseTime(),
                                           impulse);
        left_kart->getBody()->setAngularVelocity(btVector3(0,0,0));
    }

}   // KartKartCollision

//-----------------------------------------------------------------------------
/** This function is called at each internal bullet timestep. It is used
 *  here to do the collision handling: using the contact manifolds after a
 *  physics time step might miss some collisions (when more than one internal
 *  time step was done, and the collision is added and removed). So this
 *  function stores all collisions in a list, which is then handled after the
 *  actual physics timestep. This list only stores a collision if it's not
 *  already in the list, so a collisions which is reported more than once is
 *  nevertheless only handled once.
 *  The list of collision
 *  Parameters: see bullet documentation for details.
 */
btScalar Physics::solveGroup(btCollisionObject** bodies, int numBodies,
                             btPersistentManifold** manifold,int numManifolds,
                             btTypedConstraint** constraints,
                             int numConstraints,
                             const btContactSolverInfo& info,
                             btIDebugDraw* debugDrawer,
                             btStackAlloc* stackAlloc,
                             btDispatcher* dispatcher)
{
    btScalar returnValue=
        btSequentialImpulseConstraintSolver::solveGroup(bodies, numBodies,
                                                        manifold, numManifolds,
                                                        constraints,
                                                        numConstraints, info,
                                                        debugDrawer,
                                                        stackAlloc,
                                                        dispatcher);
    int currentNumManifolds = m_dispatcher->getNumManifolds();
    // We can't explode a rocket in a loop, since a rocket might collide with
    // more than one object, and/or more than once with each object (if there
    // is more than one collision point). So keep a list of rockets that will
    // be exploded after the collisions
    for(int i=0; i<currentNumManifolds; i++)
    {
        btPersistentManifold* contact_manifold =
            m_dynamics_world->getDispatcher()->getManifoldByIndexInternal(i);

        const btCollisionObject* objA =
            static_cast<const btCollisionObject*>(contact_manifold->getBody0());
        const btCollisionObject* objB =
            static_cast<const btCollisionObject*>(contact_manifold->getBody1());

        unsigned int num_contacts = contact_manifold->getNumContacts();
        if(!num_contacts) continue;   // no real collision

        const UserPointer *upA = (UserPointer*)(objA->getUserPointer());
        const UserPointer *upB = (UserPointer*)(objB->getUserPointer());

        if(!upA || !upB) continue;

        // 1) object A is a track
        // =======================
        if(upA->is(UserPointer::UP_TRACK))
        {
            if(upB->is(UserPointer::UP_FLYABLE))   // 1.1 projectile hits track
                m_all_collisions.push_back(
                    upB, contact_manifold->getContactPoint(0).m_localPointB,
                    upA, contact_manifold->getContactPoint(0).m_localPointA);
            else if(upB->is(UserPointer::UP_KART))
            {
                AbstractKart *kart=upB->getPointerKart();
                int n = contact_manifold->getContactPoint(0).m_index0;
                const Material *m
                    = n>=0 ? upA->getPointerTriangleMesh()->getMaterial(n)
                           : NULL;
                // I assume that the normal needs to be flipped in this case,
                // but  I can't verify this since it appears that bullet
                // always has the kart as object A, not B.
                const btVector3 &normal = -contact_manifold->getContactPoint(0)
                                                            .m_normalWorldOnB;
                kart->crashed(m, normal);
            }
            else if(upB->is(UserPointer::UP_PHYSICAL_OBJECT))
            {
                int n = contact_manifold->getContactPoint(0).m_index1;
                const Material *m
                    = n>=0 ? upA->getPointerTriangleMesh()->getMaterial(n)
                           : NULL;
                const btVector3 &normal = contact_manifold->getContactPoint(0)
                                                           .m_normalWorldOnB;
                upB->getPointerPhysicalObject()->hit(m, normal);
            }
        }
        // 2) object a is a kart
        // =====================
        else if(upA->is(UserPointer::UP_KART))
        {
            if(upB->is(UserPointer::UP_TRACK))
            {
                AbstractKart *kart = upA->getPointerKart();
                int n = contact_manifold->getContactPoint(0).m_index1;
                const Material *m
                    = n>=0 ? upB->getPointerTriangleMesh()->getMaterial(n)
                           : NULL;
                const btVector3 &normal = contact_manifold->getContactPoint(0)
                                                           .m_normalWorldOnB;
                kart->crashed(m, normal);   // Kart hit track
            }
            else if(upB->is(UserPointer::UP_FLYABLE))
                // 2.1 projectile hits kart
                m_all_collisions.push_back(
                    upB, contact_manifold->getContactPoint(0).m_localPointB,
                    upA, contact_manifold->getContactPoint(0).m_localPointA);
            else if (upB->is(UserPointer::UP_KART))
            {
                // 2.2 kart hits kart
                m_all_collisions.push_back(
                    upA, contact_manifold->getContactPoint(0).m_localPointA,
                    upB, contact_manifold->getContactPoint(0).m_localPointB);
            }
            else if(upB->is(UserPointer::UP_PHYSICAL_OBJECT))
                // 2.3 kart hits physical object
                m_all_collisions.push_back(
                    upB, contact_manifold->getContactPoint(0).m_localPointB,
                    upA, contact_manifold->getContactPoint(0).m_localPointA);
            else if(upB->is(UserPointer::UP_ANIMATION))
                m_all_collisions.push_back(
                    upB, contact_manifold->getContactPoint(0).m_localPointB,
                    upA, contact_manifold->getContactPoint(0).m_localPointA);
        }
        // 3) object is a projectile
        // =========================
        else if(upA->is(UserPointer::UP_FLYABLE))
        {
            // 3.1) projectile hits track
            // 3.2) projectile hits projectile
            // 3.3) projectile hits physical object
            // 3.4) projectile hits kart
            if(upB->is(UserPointer::UP_TRACK          ) ||
               upB->is(UserPointer::UP_FLYABLE        ) ||
               upB->is(UserPointer::UP_PHYSICAL_OBJECT) ||
               upB->is(UserPointer::UP_KART           )   )
            {
                m_all_collisions.push_back(
                    upA, contact_manifold->getContactPoint(0).m_localPointA,
                    upB, contact_manifold->getContactPoint(0).m_localPointB);
            }
        }
        // Object is a physical object
        // ===========================
        else if(upA->is(UserPointer::UP_PHYSICAL_OBJECT))
        {
            if(upB->is(UserPointer::UP_FLYABLE))
                m_all_collisions.push_back(
                    upB, contact_manifold->getContactPoint(0).m_localPointB,
                    upA, contact_manifold->getContactPoint(0).m_localPointA);
            else if(upB->is(UserPointer::UP_KART))
                m_all_collisions.push_back(
                    upA, contact_manifold->getContactPoint(0).m_localPointA,
                    upB, contact_manifold->getContactPoint(0).m_localPointB);
            else if(upB->is(UserPointer::UP_TRACK))
            {
                int n = contact_manifold->getContactPoint(0).m_index1;
                const Material *m
                    = n>=0 ? upB->getPointerTriangleMesh()->getMaterial(n)
                           : NULL;
                const btVector3 &normal = contact_manifold->getContactPoint(0)
                                                           .m_normalWorldOnB;
                upA->getPointerPhysicalObject()->hit(m, normal);
            }
        }
        else if (upA->is(UserPointer::UP_ANIMATION))
        {
            if(upB->is(UserPointer::UP_KART))
                m_all_collisions.push_back(
                    upA, contact_manifold->getContactPoint(0).m_localPointA,
                    upB, contact_manifold->getContactPoint(0).m_localPointB);
        }
        else
            assert("Unknown user pointer");           // 4) Should never happen
    }   // for i<numManifolds

    return returnValue;
}   // solveGroup

// ----------------------------------------------------------------------------
/** A debug draw function to show the track and all karts.
 */
void Physics::draw()
{
    if(!m_debug_drawer->debugEnabled() ||
        !World::getWorld()->isRacePhase()) return;

    video::SColor color(77,179,0,0);
    video::SMaterial material;
    material.Thickness = 2;
    material.AmbientColor = color;
    material.DiffuseColor = color;
    material.EmissiveColor= color;
    material.BackfaceCulling = false;
    material.setFlag(video::EMF_LIGHTING, false);
    irr_driver->getVideoDriver()->setMaterial(material);
    irr_driver->getVideoDriver()->setTransform(video::ETS_WORLD,
                                               core::IdentityMatrix);
    m_dynamics_world->debugDrawWorld();
    return;
}   // draw

// ----------------------------------------------------------------------------

/* EOF */

