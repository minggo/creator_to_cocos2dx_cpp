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

#include "math/Vec2.h"
#include "../Macros.h"
#include "defined.h"

#define MAX_MANIFOLD_POINTS    2

class b2Contact;

NS_CCR_BEGIN

class PhysicsCollider;

struct ManifoldPoint
{
    cocos2d::Vec2 localPoint;
    float normalImpulse;
    float tangentImpulse;
};

struct Manifold
{
    enum class Type
    {
        CIRCLE,
        FACE_A,
        FACE_B
    };
    
    ManifoldPoint points[MAX_MANIFOLD_POINTS];
    cocos2d::Vec2 localNormal;
    cocos2d::Vec2 localPoint;
    Type type;
    int pointCount;
};

struct WorldManifold
{
    cocos2d::Vec2 normal;
    cocos2d::Vec2 points[MAX_MANIFOLD_POINTS];
    float separations[MAX_MANIFOLD_POINTS];
};

class PhysicsContact
{
public:
    WorldManifold getWorldManifold() const;
    Manifold getManifold() const;
    
    void setEnabled(bool flag);
    
    bool isTouching() const;
    
    void setTangentSpeed(float speed);
    float getTangentSpeed() const;
    
    float getFriction() const;
    void setFriction(float friction);
    void resetFriction();
    
    void setRestitution(float restitution);
    float getRestitution() const;
    void resetRestitution();
    
private:
    PhysicsContact(b2Contact* b2contact);
    ~PhysicsContact();
    
    void reset();
    
    PhysicsCollider* _colliderA;
    PhysicsCollider* _colliderB;
    bool _enabled;
    bool _disabledOnce;
    bool _inverted;
    b2Contact* _b2contact;
};

NS_CCR_END
