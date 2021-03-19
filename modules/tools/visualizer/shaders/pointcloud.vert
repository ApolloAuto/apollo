/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#version 330 core

layout(location = 0) in vec4 vertPos;
uniform mat4 mvp;
out vec3 Color;

void main(void)
{
    gl_Position = mvp * vec4(vertPos.xyz, 1.0);

    float g = smoothstep(0.0, 256, vertPos.w);
    float r = 0.0;
    float b = 0.0;

    if(g <= 0.25)
    {
        r = g * 4.0;
        g = 0.0;
    }
    if(g > 0.75)
    {
        g = 0.0;
        b = g * 4.0 - 3.0; // b = g;
    }
    else
    {
        // g = g + 0.25;
//        g = g + 0.45;
        g = g + 0.35;
    }

    Color = vec3(r,g,b);
}
