/****************************************************************************
 Copyright (c) 2017 Chukong Technologies Inc.
 
 http://www.cocos2d-x.org
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/
#pragma once

#include <vector>
#include "../Macros.h"
#include "math/Vec2.h"
#include "2d/CCNode.h"

class b2World;
class b2Body;
class b2FixtureDef;

NS_CCR_BEGIN

class PhysicsCollider;

class RigidBody
{
public:
    enum class Type
    {
        STATIC,
        KINEMATIC,
        DYNAMIC,
        ANIMATED
    };
    
    struct RigidBodyInfo
    {
        cocos2d::Node* node;
        
    };
    
    cocos2d::Node* getNode() const;
    
    Type getType() const;
    void setType(Type type);
    
    bool isBullet() const;
    void setBullet(bool flag);
    
    bool isSleepingAllowed() const;
    void setSleepingAllowed(bool flag);
    
    float getGravityScale() const;
    void setGravityScale(float scale);
    
    float getLinearDumping() const;
    void setLinearDumpling(float linearDumping);
    
    float getAngularDumping() const;
    void setAngularDumping(float angularDumping);
    
    const cocos2d::Vec2& getLinearVelocity() const;
    void setLinearVelocity(const cocos2d::Vec2& linearVelocity);
    
    float getAngularVelocity() const;
    void setAngularVelocity(float angularVelocity);
    
    bool isFixedRotation() const;
    void setFixedRotation(bool flag);
    
    bool isAwake() const;
    void setAwake(bool flag);
    
    bool isActive() const;
    void setActive(bool flag);
    
    cocos2d::Vec2 getLocalPoint(const cocos2d::Vec2& worldPoint) const;
    cocos2d::Vec2 getWorldPoint(const cocos2d::Vec2& localPoint) const;
    
    cocos2d::Vec2 getLocalVector(const cocos2d::Vec2& worldVector) const;
    cocos2d::Vec2 getWorldVector(const cocos2d::Vec2& localVector) const;
    
    cocos2d::Vec2 getLocalCenter(const cocos2d::Vec2& worldCenter) const;
    cocos2d::Vec2 getWorldCenter(const cocos2d::Vec2& localCenter) const;
    
    cocos2d::Vec2 getLinearVelocityFromWorldPoint(const cocos2d::Vec2& worldPoint) const;
    
    float getMass() const;
    float getInertia() const;
    
    //TODO
    // getJointList()
    
    void applyForce(const cocos2d::Vec2& force, const cocos2d::Vec2& point, bool wake);
    void applyForceToCenter(const cocos2d::Vec2& force, bool wake);
    
    void applyTorque(float torque, bool wake);
    
    void applyLinearImpulse(const cocos2d::Vec2& impulse, const cocos2d::Vec2& point, bool wake);
    void applyAngularImpulse(float impulse, bool wake);
    
private:
    friend class PhysicsCollider;
    friend class PhysicsManager;
    
    RigidBody(b2World *world, cocos2d::Node* node, unsigned int nodeGroupIndex, Type type, bool enableContractListener,
              bool bullet, bool sleepingAllowed, float gravityScale, float linearDumping, float angularDumping,
              const cocos2d::Vec2& linearVelocity, float angularVelocity, bool fixedRotation);
    ~RigidBody();
    
    void createB2Body(b2World *world);
    
    void syncPositionToPhysics(bool animateEnabled) const;
    void syncRotationToPhysics(bool animateEnabled);
    void resetVelocity();
    float getNodeWorldRotation() const;
    b2Body* getB2body() const;
    unsigned int getNodeGroupIndex() const;
    
    CREATOR_DISALLOW_COPY_ASSIGN_AND_MOVE(RigidBody);
    
    Type _type;
    bool _enableContactListener;
    bool _bullet;
    bool _sleepingAllowed;
    float _gravityScale;
    float _linearDumping;
    float _angularDumping;
    cocos2d::Vec2 _linearVelocity;
    float _angularVelocity;
    bool _fixedRotation;
    cocos2d::Node *_node;
    unsigned int _nodeGroupIndex;
    b2Body *_b2Body;
    std::vector<PhysicsCollider*> _colliders;
};

NS_CCR_END
