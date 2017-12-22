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
#include "PhysicsCollider.h"
#include <cstdint>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include "RigidBody.h"
#include "PhysicsManager.h"
#include "defined.h"

NS_CCR_BEGIN

float PhysicsCollider::getDensity() const
{
    return _density;
}

void PhysicsCollider::setDensity(float density)
{
    _density = density;
}

float PhysicsCollider::getFriction() const
{
    return _friction;
}

void PhysicsCollider::setFriction(float friction)
{
    friction = friction;
}

bool PhysicsCollider::isSensor() const
{
    return _sensor;
}

void PhysicsCollider::setSensor(bool flag)
{
    _sensor = flag;
}

float PhysicsCollider::getRestitution() const
{
    return _restitution;
}

void PhysicsCollider::setRestitution(float restitution)
{
    _restitution = restitution;
}

void PhysicsCollider::apply()
{
    destroy();
    init();
}

cocos2d::Rect PhysicsCollider::getAABB() const
{
    // TODO
    return cocos2d::Rect(0,0,0,0);
}

//
// Private functions
//
PhysicsCollider::PhysicsCollider(RigidBody* rigidBody, const std::vector<bool>& collisionMatrix, const ColliderInfoCommon& colliderInfo)
    : _rigidBody(rigidBody)
    , _density(colliderInfo.density)
    , _friction(colliderInfo.friction)
    , _restitution(colliderInfo.restitution)
    , _sensor(colliderInfo.sensor)
    , _offset(colliderInfo.offset)
    , _inited(false)
    , _collisionMatrix(std::move(collisionMatrix))
{
    init();
}

PhysicsCollider::~PhysicsCollider()
{
    destroy();

    _rigidBody = nullptr;
}

void PhysicsCollider::init()
{
    if (_inited)
        return;
    
    auto node = _rigidBody->getNode();
    auto nodeScale = getNodeWorldScale(node);
    
    if (nodeScale != 0)
    {
        auto shapes = createShapes(nodeScale);
        auto nodeGroupIndex = _rigidBody->getNodeGroupIndex();
        uint16_t categoryBits = 1 << nodeGroupIndex;
        uint16_t maskBits = 0;
        for (size_t i = 0, len = _collisionMatrix.size(); i < len; ++i)
        {
            if (_collisionMatrix[i])
                maskBits |= 1 << i;
        }
        
        b2Filter filter;
        filter.categoryBits = categoryBits;
        filter.maskBits = maskBits;
        filter.groupIndex = 0;
        
        auto b2body = _rigidBody->getB2body();
        for (auto& shape : shapes)
        {
            b2FixtureDef fixtureDef;
            fixtureDef.density = _density;
            fixtureDef.isSensor = _sensor;
            fixtureDef.friction = _friction;
            fixtureDef.restitution = _restitution;
            fixtureDef.shape = shape;
            fixtureDef.filter = filter;
            
            auto fixture = b2body->CreateFixture(&fixtureDef);
            delete shape;
            
            // Connet fixture with PhysicsCollider.
            fixture->SetUserData(this);
            
            auto physicsManager = PhysicsManager::getInstance();
            if (_rigidBody->_enableContactListener)
                physicsManager->registerContactFixture(fixture);
            
            _fixtures.push_back(fixture);
        }
    }
    
    _inited = true;
}

void PhysicsCollider::destroy()
{
    if (!_inited)
        return;
    
    auto b2body = _rigidBody->getB2body();
    auto physicsManager = PhysicsManager::getInstance();
    for (auto& fixture : _fixtures)
    {
        fixture->SetUserData(nullptr);
        physicsManager->unregisterContactFixture(fixture);
        b2body->DestroyFixture(fixture);
    }
    
    _fixtures.clear();
    _inited = false;
}

float PhysicsCollider::getNodeWorldScale(const cocos2d::Node* node) const
{
    auto scale = node->getScale();
    auto parent = node->getParent();
    while (parent)
    {
        scale *= parent->getScale();
        parent = parent->getParent();
    }
    
    return scale;
}

RigidBody* PhysicsCollider::getBody() const
{
    return _rigidBody;
}

//
// PhysicsColliderBox
//

PhysicsColliderBox::PhysicsColliderBox(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix, const struct ColliderInfo& colliderInfo)
    : _size(colliderInfo.size)
    , PhysicsCollider(rigidBody, collisionMatrix, colliderInfo)
{
}

std::vector<b2Shape*> PhysicsColliderBox::createShapes(float scale)
{
    auto scaleAbs = std::abs(scale);
    auto width = _size.width / 2 / PTM_RATIO * scaleAbs;
    auto height = _size.height / 2 / PTM_RATIO * scaleAbs;
    auto offsetX = _offset.x / PTM_RATIO * scale;
    auto offsetY = _offset.y / PTM_RATIO * scale;

    auto shape = new b2PolygonShape();
    shape->SetAsBox(width, height, b2Vec2(offsetX, offsetY), 0);
    
    std::vector<b2Shape*> shapes{shape};
    return shapes;
}

//
// PhysicsColliderCircle
//
PhysicsColliderCircle::PhysicsColliderCircle(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix, const struct ColliderInfo& colliderInfo)
    : _radius(colliderInfo.radius)
    , PhysicsCollider(rigidBody, collisionMatrix, colliderInfo)
{
}
          
std::vector<b2Shape*> PhysicsColliderCircle::createShapes(float scale)
{
    auto scaleAbs = std::abs(scale);
    auto offsetX = _offset.x / PTM_RATIO * scale;
    auto offsetY = _offset.y / PTM_RATIO * scale;
    
    auto shape = new b2CircleShape();
    shape->m_radius = _radius / PTM_RATIO * scaleAbs;
    shape->m_p = b2Vec2(offsetX, offsetY);
    
    std::vector<b2Shape*> shapes{shape};
    return shapes;
}

//
// PhysicsColliderChain
//
PhysicsColliderChain::PhysicsColliderChain(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix,
                                               const struct ColliderInfo& colliderInfo)
: _loop(colliderInfo.loop)
, _points(std::move(colliderInfo.points))
, creator::PhysicsCollider(rigidBody, collisionMatrix, colliderInfo)
{
}

std::vector<b2Shape*> PhysicsColliderChain::createShapes(float scale)
{
    auto pointCount = _points.size();
    b2Vec2 vertices[pointCount];
    for (size_t i = 0; i < pointCount; ++i)
    {
        auto point = _points[i];
        vertices[i] = b2Vec2(point.x / PTM_RATIO * scale, point.y / PTM_RATIO * scale);
    }
    
    auto shape = new b2ChainShape();
    if (_loop)
        shape->CreateLoop(vertices, pointCount);
    else
        shape->CreateChain(vertices, pointCount);
    
    std::vector<b2Shape*> shapes{shape};
    return shapes;
}

//
// PhysicsColliderPolygon
//
//TODO

NS_CCR_END
