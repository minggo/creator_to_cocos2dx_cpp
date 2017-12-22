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

#include "PhysicsContact.h"
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Collision/b2Collision.h>
#include "PhysicsCollider.h"

NS_CCR_BEGIN

namespace
{
    void inline b2ManifoldPoint2ManifoldPoint(const b2ManifoldPoint& b2manifoldPoint, ManifoldPoint& out)
    {
        out.localPoint = B2VEC_POINT_TO_COCOS2D_VEC(b2manifoldPoint.localPoint);
        out.normalImpulse = b2manifoldPoint.normalImpulse;
        out.tangentImpulse = b2manifoldPoint.tangentImpulse;
    }
}

WorldManifold PhysicsContact::getWorldManifold() const
{
    b2WorldManifold b2worldManifold;
    _b2contact->GetWorldManifold(&b2worldManifold);
    
    WorldManifold worldManifold;
    worldManifold.normal = B2VEC_TO_COCOS2D_VEC(b2worldManifold.normal);
    for (int i = 0; i < MAX_MANIFOLD_POINTS; ++i)
    {
        worldManifold.points[i] = B2VEC_POINT_TO_COCOS2D_VEC(b2worldManifold.points[i]);
        worldManifold.separations[i] = b2worldManifold.separations[i] * PTM_RATIO;
    }
    
    if (_inverted)
        worldManifold.normal = -worldManifold.normal;
    
    return worldManifold;
}

Manifold PhysicsContact::getManifold() const
{
    auto manifold = _b2contact->GetManifold();
    Manifold ret;
    ret.localNormal = B2VEC_TO_COCOS2D_VEC(manifold->localNormal);
    ret.localPoint = B2VEC_POINT_TO_COCOS2D_VEC(manifold->localNormal);
    ret.pointCount = manifold->pointCount;
    ret.type = static_cast<Manifold::Type>(manifold->type);
    
    // Use memory copy to speed up?
    for (int i = 0, count = manifold->pointCount; i < count; ++i)
        b2ManifoldPoint2ManifoldPoint(manifold->points[i], ret.points[i]);
    
    if (_inverted)
        ret.localNormal = -ret.localNormal;
    
    return ret;
}

void PhysicsContact::setEnabled(bool flag)
{
    _b2contact->SetEnabled(flag);
}

bool PhysicsContact::isTouching() const
{
    return _b2contact->IsTouching();
}

void PhysicsContact::setTangentSpeed(float speed)
{
    _b2contact->SetTangentSpeed(speed);
}

float PhysicsContact::getTangentSpeed() const
{
    return _b2contact->GetTangentSpeed();
}

float PhysicsContact::getFriction() const
{
    return _b2contact->GetFriction();
}

void PhysicsContact::setFriction(float friction)
{
    _b2contact->SetFriction(friction);
}

void PhysicsContact::resetFriction()
{
    _b2contact->ResetFriction();
}

void PhysicsContact::setRestitution(float restitution)
{
    _b2contact->SetRestitution(restitution);
}

float PhysicsContact::getRestitution() const
{
    return _b2contact->GetRestitution();
}

void PhysicsContact::resetRestitution()
{
    _b2contact->ResetRestitution();
}

//
// private functions
//

PhysicsContact::PhysicsContact(b2Contact* b2contact)
: _enabled(true)
, _disabledOnce(false)
, _inverted(false)
, _b2contact(b2contact)
{
    _colliderA = static_cast<PhysicsCollider*>(_b2contact->GetFixtureA()->GetUserData());
    _colliderB = static_cast<PhysicsCollider*>(_b2contact->GetFixtureB()->GetUserData());
}

NS_CCR_END
