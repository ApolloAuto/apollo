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

layout(location = 0) in vec2 vertPos;
layout(location = 1) in vec2 texCoord;

out vec2 TexCoord;

void main(void)
{
    gl_Position = vec4(vertPos.x, vertPos.y, 0.0, 1.0);
    TexCoord = vec2(texCoord.x, 1.0 - texCoord.y);
}
