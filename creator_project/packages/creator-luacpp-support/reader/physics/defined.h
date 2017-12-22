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

#include "base/ccMacros.h"

#define PTM_RATIO 32
#define CREATOR_DEGREES_TO_PHYSICS_RADIANS(__ANGLE__) -((__ANGLE__) * 0.01745329252f) // - (PI / 180)

#define B2VEC_POINT_TO_COCOS2D_VEC(b2vec)     {(b2vec).x * PTM_RATIO, b2vec.y * PTM_RATIO}
#define B2VEC_TO_COCOS2D_VEC(b2vec)           {(b2vec).x, (b2vec).y}
#define COCOS2D_VEC_TO_B2VEC_POINT(cocosvec)  {(cocosvec).x / PTM_RATIO, (cocosvec).y / PTM_RATIO}
#define COCOS2D_VEC_TO_B2VEC_(cocosvec)       {(cocosvec).x, (cocosvec).y}
