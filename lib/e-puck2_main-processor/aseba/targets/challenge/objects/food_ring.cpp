/*
	Challenge - Virtual Robot Challenge System
	Copyright (C) 1999--2008:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
	3D models
	Copyright (C) 2008:
		Basilio Noris
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2016:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <QtOpenGL>
#define BYTE unsigned char

namespace Enki
{
	// 48 Verticies
	// 50 Texture Coordinates
	// 48 Normals
	// 48 Triangles
	
	static BYTE face_indicies[48][9] = {
	// Object #-1
		{2,24,1 ,0,1,2 ,0,1,2 }, {2,25,24 ,0,3,1 ,0,3,1 }, {3,25,2 ,4,3,0 ,4,3,0 },
		{3,26,25 ,4,5,3 ,4,5,3 }, {4,26,3 ,6,5,4 ,6,5,4 }, {4,27,26 ,6,7,5 ,6,7,5 },
		{5,27,4 ,8,7,6 ,8,7,6 }, {5,28,27 ,8,9,7 ,8,9,7 }, {6,28,5 ,10,9,8 ,10,9,8 },
		{6,29,28 ,10,11,9 ,10,11,9 }, {7,29,6 ,12,11,10 ,12,11,10 },
		{7,30,29 ,12,13,11 ,12,13,11 }, {8,30,7 ,14,13,12 ,14,13,12 },
		{8,31,30 ,14,15,13 ,14,15,13 }, {9,31,8 ,16,15,14 ,16,15,14 },
		{9,32,31 ,16,17,15 ,16,17,15 }, {10,32,9 ,18,17,16 ,18,17,16 },
		{10,33,32 ,18,19,17 ,18,19,17 }, {11,33,10 ,20,19,18 ,20,19,18 },
		{11,34,33 ,20,21,19 ,20,21,19 }, {12,34,11 ,22,21,20 ,22,21,20 },
		{12,35,34 ,22,23,21 ,22,23,21 }, {13,35,12 ,24,23,22 ,24,23,22 },
		{13,36,35 ,24,25,23 ,24,25,23 }, {14,36,13 ,26,25,24 ,26,25,24 },
		{14,37,36 ,26,27,25 ,26,27,25 }, {15,37,14 ,28,27,26 ,28,27,26 },
		{15,38,37 ,28,29,27 ,28,29,27 }, {16,38,15 ,30,29,28 ,30,29,28 },
		{16,39,38 ,30,31,29 ,30,31,29 }, {17,39,16 ,32,31,30 ,32,31,30 },
		{17,40,39 ,32,33,31 ,32,33,31 }, {18,40,17 ,34,33,32 ,34,33,32 },
		{18,41,40 ,34,35,33 ,34,35,33 }, {19,41,18 ,36,35,34 ,36,35,34 },
		{19,42,41 ,36,37,35 ,36,37,35 }, {20,42,19 ,38,37,36 ,38,37,36 },
		{20,43,42 ,38,39,37 ,38,39,37 }, {21,43,20 ,40,39,38 ,40,41,42 },
		{21,44,43 ,40,41,39 ,40,43,41 }, {22,44,21 ,42,41,40 ,44,43,40 },
		{22,45,44 ,42,43,41 ,44,45,43 }, {23,45,22 ,44,43,42 ,46,45,44 },
		{23,46,45 ,44,45,43 ,46,47,45 }, {0,46,23 ,46,45,44 ,48,47,46 },
		{0,47,46 ,46,47,45 ,48,49,47 }, {1,47,0 ,2,47,46 ,2,49,48 },
		{1,24,47 ,2,1,47 ,2,1,49 }
	};
	static GLfloat vertices [48][3] = {
	{-0.0225f,-0.0389711f,0.00525f},{-0.0116469f,-0.0434667f,0.00525f},{0.0f,-0.045f,0.00525f},
	{0.0116469f,-0.0434667f,0.00525f},{0.0225f,-0.0389711f,0.00525f},{0.0318198f,-0.0318198f,0.00525f},
	{0.0389711f,-0.0225f,0.00525f},{0.0434667f,-0.0116469f,0.00525f},{0.045f,0.0f,0.00525f},
	{0.0434667f,0.0116469f,0.00525f},{0.0389711f,0.0225f,0.00525f},{0.0318198f,0.0318198f,0.00525f},
	{0.0225f,0.0389711f,0.00525f},{0.0116469f,0.0434667f,0.00525f},{0.0f,0.045f,0.00525f},
	{-0.0116469f,0.0434667f,0.00525f},{-0.0225f,0.0389711f,0.00525f},{-0.0318198f,0.0318198f,0.00525f},
	{-0.0389711f,0.0225f,0.00525f},{-0.0434667f,0.0116469f,0.00525f},{-0.045f,0.0f,0.00525f},
	{-0.0434667f,-0.0116469f,0.00525f},{-0.0389711f,-0.0225f,0.00525f},{-0.0318198f,-0.0318198f,0.00525f},
	{-0.0116469f,-0.0434667f,0.045875f},{0.0f,-0.045f,0.045875f},{0.0116469f,-0.0434667f,0.045875f},
	{0.0225f,-0.0389711f,0.045875f},{0.0318198f,-0.0318198f,0.045875f},{0.0389711f,-0.0225f,0.045875f},
	{0.0434667f,-0.0116469f,0.045875f},{0.045f,0.0f,0.045875f},{0.0434667f,0.0116469f,0.045875f},
	{0.0389711f,0.0225f,0.045875f},{0.0318198f,0.0318198f,0.045875f},{0.0225f,0.0389711f,0.045875f},
	{0.0116469f,0.0434667f,0.045875f},{0.0f,0.045f,0.045875f},{-0.0116469f,0.0434667f,0.045875f},
	{-0.0225f,0.0389711f,0.045875f},{-0.0318198f,0.0318198f,0.045875f},{-0.0389711f,0.0225f,0.045875f},
	{-0.0434667f,0.0116469f,0.045875f},{-0.045f,0.0f,0.045875f},{-0.0434667f,-0.0116469f,0.045875f},
	{-0.0389711f,-0.0225f,0.045875f},{-0.0318198f,-0.0318198f,0.045875f},{-0.0225f,-0.0389711f,0.045875f}
	};
	static GLfloat normals [48][3] = {
	{-0.0438419f,-0.999038f,0.0f},{-0.216222f,-0.976344f,0.0f},{-0.300918f,-0.95365f,0.0f},
	{0.0438419f,-0.999038f,0.0f},{0.216222f,-0.976344f,0.0f},{0.300918f,-0.95365f,0.0f},
	{0.461551f,-0.887114f,0.0f},{0.537487f,-0.843272f,0.0f},{0.675426f,-0.737428f,0.0f},
	{0.737428f,-0.675426f,0.0f},{0.843272f,-0.537487f,0.0f},{0.887114f,-0.461551f,0.0f},
	{0.95365f,-0.300918f,0.0f},{0.976344f,-0.216222f,0.0f},{0.999038f,-0.0438419f,0.0f},
	{0.999038f,0.0438419f,0.0f},{0.976344f,0.216222f,0.0f},{0.95365f,0.300918f,0.0f},
	{0.887114f,0.461551f,0.0f},{0.843272f,0.537487f,0.0f},{0.737428f,0.675426f,0.0f},
	{0.675426f,0.737428f,0.0f},{0.537487f,0.843272f,0.0f},{0.461551f,0.887114f,0.0f},
	{0.300918f,0.95365f,0.0f},{0.216222f,0.976344f,0.0f},{0.0438419f,0.999038f,0.0f},
	{-0.0438419f,0.999038f,0.0f},{-0.216222f,0.976344f,0.0f},{-0.300918f,0.95365f,0.0f},
	{-0.461551f,0.887114f,0.0f},{-0.537487f,0.843272f,0.0f},{-0.675426f,0.737428f,0.0f},
	{-0.737428f,0.675426f,0.0f},{-0.843272f,0.537487f,0.0f},{-0.887114f,0.461551f,0.0f},
	{-0.95365f,0.300918f,0.0f},{-0.976344f,0.216222f,0.0f},{-0.999038f,0.0438419f,0.0f},
	{-0.999038f,-0.0438419f,0.0f},{-0.976344f,-0.216222f,0.0f},{-0.95365f,-0.300918f,0.0f},
	{-0.887114f,-0.461551f,0.0f},{-0.843272f,-0.537487f,0.0f},{-0.737428f,-0.675426f,0.0f},
	{-0.675426f,-0.737428f,0.0f},{-0.537487f,-0.843272f,0.0f},{-0.461551f,-0.887114f,0.0f}
	};
	static GLfloat textures [50][2] = {
	{0.251939f,0.326379f},{0.210595f,0.428389f},{0.210595f,0.326379f},
	{0.251939f,0.428389f},{0.293282f,0.326379f},{0.293282f,0.428389f},
	{0.334626f,0.326379f},{0.334626f,0.428389f},{0.375969f,0.326379f},
	{0.375969f,0.428389f},{0.417313f,0.326379f},{0.417313f,0.428389f},
	{0.458656f,0.326379f},{0.458656f,0.428389f},{0.5f,0.326379f},
	{0.5f,0.428389f},{0.541343f,0.326379f},{0.541343f,0.428389f},
	{0.582687f,0.326379f},{0.582687f,0.428389f},{0.62403f,0.326379f},
	{0.62403f,0.428389f},{0.665374f,0.326379f},{0.665374f,0.428389f},
	{0.706717f,0.326379f},{0.706717f,0.428389f},{0.748061f,0.326379f},
	{0.748061f,0.428389f},{0.789404f,0.326379f},{0.789404f,0.428389f},
	{0.830748f,0.326379f},{0.830748f,0.428389f},{0.872092f,0.326379f},
	{0.872092f,0.428389f},{0.913435f,0.326379f},{0.913435f,0.428389f},
	{0.954778f,0.326379f},{0.954778f,0.428389f},{0.996122f,0.326379f},
	{0.996122f,0.428389f},{0.0452214f,0.326379f},{0.00387793f,0.427763f},
	{0.00387793f,0.326379f},{0.0452214f,0.428389f},{0.086565f,0.326379f},
	{0.086565f,0.428389f},{0.127908f,0.326379f},{0.127908f,0.428389f},
	{0.169252f,0.326379f},{0.169252f,0.428389f}
	};

	GLint GenFeederRing()
	{
		GLint lid=glGenLists(1);
		glNewList(lid, GL_COMPILE);
		
		glBegin (GL_TRIANGLES);
		for(size_t i=0;i<sizeof(face_indicies)/sizeof(face_indicies[0]);i++)
		{
			for(size_t j=0;j<3;j++)
			{
				int vi=face_indicies[i][j];
				int ni=face_indicies[i][j+3];//Normal index
				int ti=face_indicies[i][j+6];//Texture index
				/*glNormal3f (normals[ni][0],normals[ni][1],normals[ni][2]);
				glTexCoord2f(textures[ti][0],textures[ti][1]);
				glVertex3f (vertices[vi][0],vertices[vi][1],vertices[vi][2]);*/
				
				// rotate 90 deg around z
				glNormal3f (normals[ni][1],-normals[ni][0],normals[ni][2]);
				glTexCoord2f(textures[ti][0],textures[ti][1]);
				glVertex3f (100.f*vertices[vi][1],-100.f*vertices[vi][0],100.f*vertices[vi][2]);
			}
		}
		glEnd ();
		
		glEndList();
		return lid;
	};
}
