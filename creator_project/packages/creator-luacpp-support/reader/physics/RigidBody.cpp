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

#include "RigidBody.h"
#include <Box2D/Box2D.h>
#include "base/CCDirector.h"
#include "defined.h"

NS_CCR_BEGIN

RigidBody::RigidBody(b2World *world, cocos2d::Node* node, unsigned int nodeGroupIndex, Type type, bool enableContractListener,
                     bool bullet, bool sleepingAllowed, float gravityScale, float linearDumping, float angularDumping,
                     const cocos2d::Vec2& linearVelocity, float angularVelocity, bool fixedRotation)
    : _node(node)
    , _type(type)
    , _enableContactListener(enableContractListener)
    , _bullet(bullet)
    , _sleepingAllowed(sleepingAllowed)
    , _gravityScale(gravityScale)
    , _linearDumping(linearDumping)
    , _angularDumping(angularDumping)
    , _linearVelocity(linearVelocity)
    , _angularVelocity(angularVelocity)
    , _fixedRotation(fixedRotation)
    , _nodeGroupIndex(nodeGroupIndex)
{
    assert(node);
    _node->retain();
    
    createB2Body(world);
}

RigidBody::~RigidBody()
{
    _node->release();
    _node = nullptr;
}

cocos2d::Node* RigidBody::getNode() const
{
    return _node;
}

RigidBody::Type RigidBody::getType() const
{
    return _type;
}

void RigidBody::setType(Type type)
{
    _type = type;
    
    if (RigidBody::Type::ANIMATED == _type)
        _b2Body->SetType(b2BodyType::b2_kinematicBody);
    else
        _b2Body->SetType(static_cast<b2BodyType>(_type));
}

bool RigidBody::isBullet() const
{
    return _bullet;
}

void RigidBody::setBullet(bool flag)
{
    _bullet = flag;
    _b2Body->SetBullet(_bullet);
}

bool RigidBody::isSleepingAllowed() const
{
    return _sleepingAllowed;
}

void RigidBody::setSleepingAllowed(bool flag)
{
    _sleepingAllowed = flag;
    _b2Body->SetSleepingAllowed(_sleepingAllowed);
}

float RigidBody::getGravityScale() const
{
    return _gravityScale;
}

void RigidBody::setGravityScale(float scale)
{
    _gravityScale = scale;
    _b2Body->SetGravityScale(_gravityScale);
}

float RigidBody::getLinearDumping() const
{
    return _linearDumping;
}

void RigidBody::setLinearDumpling(float linearDumping)
{
    _linearDumping = linearDumping;
    _b2Body->SetLinearDamping(_linearDumping);
}

float RigidBody::getAngularDumping() const
{
    return _angularDumping;
}

void RigidBody::setAngularDumping(float angularDumping)
{
    _angularDumping = angularDumping;
    _b2Body->SetAngularDamping(_angularDumping);
}

const cocos2d::Vec2& RigidBody::getLinearVelocity() const
{
    return _linearVelocity;
}

void RigidBody::setLinearVelocity(const cocos2d::Vec2& linearVelocity)
{
    _linearVelocity = linearVelocity;
    _b2Body->SetLinearVelocity({_linearVelocity.x, _linearVelocity.y});
}

float RigidBody::getAngularVelocity() const
{
    return _angularVelocity;
}

void RigidBody::setAngularVelocity(float angularVelocity)
{
    _angularVelocity = angularVelocity;
    _b2Body->SetAngularVelocity(_angularVelocity);
}

bool RigidBody::isFixedRotation() const
{
    return _fixedRotation;
}

void RigidBody::setFixedRotation(bool flag)
{
    _fixedRotation = flag;
    _b2Body->SetFixedRotation(_fixedRotation);
}


bool RigidBody::isAwake() const
{
    return _b2Body->IsAwake();
}

void RigidBody::setAwake(bool flag)
{
    _b2Body->SetAwake(flag);
}

bool RigidBody::isActive() const
{
    return _b2Body->IsActive();
}

void RigidBody::setActive(bool flag)
{
    _b2Body->SetActive(flag);
}

float RigidBody::getMass() const
{
    return _b2Body->GetMass();
}

float RigidBody::getInertia() const
{
    return _b2Body->GetInertia();
}

//
//cocos2d::Vec2 getLocalPoint(const cocos2d::Vec2& worldPoint) const;
//cocos2d::Vec2 getWorldPoint(const cocos2d::Vec2& localPoint) const;
//
//cocos2d::Vec2 getLocalVector(const cocos2d::Vec2& worldVector) const;
//cocos2d::Vec2 getWorldVector(const cocos2d::Vec2& localVector) const;
//
//cocos2d::Vec2 getLocalCenter(const cocos2d::Vec2& worldCenter) const;
//cocos2d::Vec2 getWorldCenter(const cocos2d::Vec2& localCenter) const;
//
//cocos2d::Vec2 getLinearVelocityFromWorldPoint(const cocos2d::Vec2& worldPoint) const;
//

//
////TODO
//// getJointList()
//
void RigidBody::applyForce(const cocos2d::Vec2& force, const cocos2d::Vec2& point, bool wake)
{
    b2Vec2 physicsForce(force.x / PTM_RATIO, force.y / PTM_RATIO);
    b2Vec2 phsicsPoint(point.x / PTM_RATIO, point.y / PTM_RATIO);
    _b2Body->ApplyForce(physicsForce, phsicsPoint, wake);
}

void RigidBody::applyForceToCenter(const cocos2d::Vec2& force, bool wake)
{
    b2Vec2 physicsForce(force.x / PTM_RATIO, force.y / PTM_RATIO);
    _b2Body->ApplyForceToCenter(physicsForce, wake);
}

void RigidBody::applyTorque(float torque, bool wake)
{
    _b2Body->ApplyTorque(torque / PTM_RATIO, wake);
}

void RigidBody::applyLinearImpulse(const cocos2d::Vec2& impulse, const cocos2d::Vec2& point, bool wake)
{
    b2Vec2 physicsImpulse(impulse.x / PTM_RATIO, impulse.y / PTM_RATIO);
    b2Vec2 physicsPoint(point.x / PTM_RATIO, point.y / PTM_RATIO);
    _b2Body->ApplyLinearImpulse(physicsImpulse, physicsPoint, wake);
}

void RigidBody::applyAngularImpulse(float impulse, bool wake)
{
    _b2Body->ApplyAngularImpulse(impulse / PTM_RATIO / PTM_RATIO, wake);
}

// private functions

void RigidBody::resetVelocity()
{
    _b2Body->SetLinearVelocity(b2Vec2(0, 0));
    _b2Body->SetAngularVelocity(0);
}

void RigidBody::createB2Body(b2World *world)
{
    b2BodyDef bodyDef;
    bodyDef.type = static_cast<b2BodyType>(_type);
    bodyDef.allowSleep = _sleepingAllowed;
    bodyDef.linearDamping = _linearDumping;
    bodyDef.angularDamping = _angularDumping;
    bodyDef.bullet = _bullet;
    bodyDef.gravityScale = _gravityScale;
    bodyDef.angularVelocity = _angularVelocity;
    bodyDef.linearVelocity = {_linearVelocity.x, _linearVelocity.y};
    bodyDef.fixedRotation = _fixedRotation;
    
    _b2Body = world->CreateBody(&bodyDef);
}

void RigidBody::syncPositionToPhysics(bool animateEnabled) const
{
    auto nodePos(_node->convertToWorldSpaceAR(cocos2d::Vec2::ZERO));
    b2Vec2 targetPos(nodePos.x / PTM_RATIO, nodePos.y / PTM_RATIO);
    
    // Set linear velocity instead if rigid body type is ANIMATED and animate is enabled.
    if (RigidBody::Type::ANIMATED == _type && animateEnabled)
    {
        auto currentPos = _b2Body->GetPosition();
        
        auto frameRate = 1.f / cocos2d::Director::getInstance()->getFrameRate();
        b2Vec2 targetLinearVelocity((targetPos.x - currentPos.x) * frameRate,
                                    (targetPos.y - currentPos.y) * frameRate);
        _b2Body->SetAwake(true);
        _b2Body->SetLinearVelocity(targetLinearVelocity);
    }
    else
        _b2Body->SetTransform(targetPos, _b2Body->GetAngle());
}

void RigidBody::syncRotationToPhysics(bool animateEnabled)
{
    auto targetRotation = CREATOR_DEGREES_TO_PHYSICS_RADIANS(getNodeWorldRotation());
    
    // Set angular velocity instead if rigid body type is ANIMATED and animate is enabled.
    if (RigidBody::Type::ANIMATED == _type && animateEnabled)
    {
        auto currentRotation = _b2Body->GetAngle();
        auto frameRate = 1.f / cocos2d::Director::getInstance()->getFrameRate();
        _b2Body->SetAwake(true);
        _b2Body->SetAngularVelocity((targetRotation - currentRotation) * frameRate);
    }
    else
        _b2Body->SetTransform(_b2Body->GetPosition(), targetRotation);
}

float RigidBody::getNodeWorldRotation() const
{
    auto rotation = _node->getRotation();
    auto parent = _node->getParent();
    while (parent)
    {
        rotation += parent->getRotation();
        parent = parent->getParent();
    }
    
    return rotation;
}

b2Body* RigidBody::getB2body() const
{
    return _b2Body;
}

unsigned int RigidBody::getNodeGroupIndex() const
{
    return _nodeGroupIndex;
}

NS_CCR_END
