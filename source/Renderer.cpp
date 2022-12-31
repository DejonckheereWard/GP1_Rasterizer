//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Texture.h"
#include "Utils.h"
#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window* pWindow):
	m_pWindow(pWindow)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);

	//Create Buffers
	m_pFrontBuffer = SDL_GetWindowSurface(pWindow);
	m_pBackBuffer = SDL_CreateRGBSurface(0, m_Width, m_Height, 32, 0, 0, 0, 0);
	m_pBackBufferPixels = (uint32_t*)m_pBackBuffer->pixels;

	m_pDepthBufferPixels = new float[m_Width * m_Height];


	//Initialize Camera
	m_Camera.Initialize(45.f, { .0f,0.0f,0.f }, m_Width / (float)m_Height);

	// Temp texture init
	m_pTexture = Texture::LoadFromFile("./Resources/vehicle_diffuse.png");
	m_pNormal = Texture::LoadFromFile("./Resources/vehicle_normal.png");
	m_pSpecular = Texture::LoadFromFile("./Resources/vehicle_specular.png");
	m_pGloss = Texture::LoadFromFile("./Resources/vehicle_gloss.png");

	Mesh& mesh = m_Meshes.emplace_back(Mesh{});
	Utils::ParseOBJ("Resources/vehicle.obj", mesh.vertices, mesh.indices);
	mesh.primitiveTopology = PrimitiveTopology::TriangleList;
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;
	delete m_pTexture;
}

void Renderer::Update(Timer* pTimer)
{
	m_Camera.Update(pTimer);


	if(m_RotationEnabled)
	{
		m_CurrentMeshRotation += 0.5f * pTimer->GetElapsed();
		for(Mesh& mesh : m_Meshes)
		{
			Matrix translationMatrix = Matrix::CreateTranslation(0.f, 0.f, 50.f);
			Matrix rotationMatrix = Matrix::CreateRotationY(m_CurrentMeshRotation);
			mesh.worldMatrix = rotationMatrix * translationMatrix;
		}
	}
}

void Renderer::Render()
{
	//@START
	//Lock BackBuffer
	SDL_LockSurface(m_pBackBuffer);

	//Render_W06_P1();
	//Render_W06_P2();
	//Render_W06_P3();
	//Render_W06_P4();
	//Render_W06_P5();

	Render_W08_P1();


	//RENDER LOGIC
	//for (int px{}; px < m_Width; ++px)
	//{
	//	for (int py{}; py < m_Height; ++py)
	//	{
	//		float gradient = px / static_cast<float>(m_Width);
	//		gradient += py / static_cast<float>(m_Width);
	//		gradient /= 2.0f;

	//		ColorRGB finalColor{ gradient, gradient, gradient };

	//		//Update Color in Buffer
	//		finalColor.MaxToOne();

	//		m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
	//			static_cast<uint8_t>(finalColor.r * 255),
	//			static_cast<uint8_t>(finalColor.g * 255),
	//			static_cast<uint8_t>(finalColor.b * 255));
	//	}
	//}

	//@END
	//Update SDL Surface
	SDL_UnlockSurface(m_pBackBuffer);
	SDL_BlitSurface(m_pBackBuffer, 0, m_pFrontBuffer, 0);
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const
{
	// Convert the vertices_in from world space to screen space (NDC intermedian step)
	// World -> NDC -> Screen

	vertices_out.clear();
	vertices_out.reserve(vertices_in.size());

	for(const Vertex& vert : vertices_in)
	{
		// World to camera (View Space)
		Vector3 viewSpaceVertex = m_Camera.viewMatrix.TransformPoint(vert.position);  // Transform the position

		// Consider the camera (fov & aspect ration) first
		Vector3 adjustedViewSpaceVertex;
		const float aspectRatio = m_Width / (float)m_Height;
		adjustedViewSpaceVertex.x = viewSpaceVertex.x / (aspectRatio * m_Camera.fov);
		adjustedViewSpaceVertex.y = viewSpaceVertex.y / m_Camera.fov;

		// Convert to NDC (perspective divide) / projection space
		Vector3 ndcSpaceVertex;
		ndcSpaceVertex.x = adjustedViewSpaceVertex.x / viewSpaceVertex.z;
		ndcSpaceVertex.y = adjustedViewSpaceVertex.y / viewSpaceVertex.z;
		ndcSpaceVertex.z = viewSpaceVertex.z;

		// Transform to screen space
		Vertex& screenSpaceVert = vertices_out.emplace_back(Vertex{});
		screenSpaceVert.color = vert.color;  // Copy the color
		screenSpaceVert.position = {
			(ndcSpaceVertex.x + 1) / 2.0f * m_Width , // Screen X
			(1 - ndcSpaceVertex.y) / 2.0f * m_Height, // Screen Y,
			ndcSpaceVertex.z
		};
	}
}


void Renderer::VertexTransformationFunction(std::vector<Mesh>& meshes) const
{
	// Convert the vertices_in from world space to screen space (NDC intermedian step)
	// World -> NDC -> Screen

#pragma region Week07
	//for (Mesh& mesh : meshes)
	//{
	//	for (const Vertex& vert : mesh.vertices)
	//	{
	//		// World to camera (view space)
	//		Vector3 viewSpaceVertex = m_Camera.viewMatrix.TransformPoint(vert.position);  // Transform the position

	//		// Consider the camera (fov & aspect ration) first
	//		Vector3 adjustedViewSpaceVertex;
	//		const float aspectRatio = m_Width / (float)m_Height;
	//		adjustedViewSpaceVertex.x = viewSpaceVertex.x / (aspectRatio * m_Camera.fov);
	//		adjustedViewSpaceVertex.y = viewSpaceVertex.y / m_Camera.fov;

	//		// Convert to NDC (perspective divide) / projection space
	//		Vector3 ndcSpaceVertex;
	//		ndcSpaceVertex.x = adjustedViewSpaceVertex.x / viewSpaceVertex.z;
	//		ndcSpaceVertex.y = adjustedViewSpaceVertex.y / viewSpaceVertex.z;
	//		ndcSpaceVertex.z = viewSpaceVertex.z;

	//		// Transform to screen space
	//		Vertex_Out& screenSpaceVert = mesh.vertices_out.emplace_back(Vertex_Out{});
	//		screenSpaceVert.position = {
	//			(ndcSpaceVertex.x + 1) / 2.0f * m_Width , // Screen X
	//			(1 - ndcSpaceVertex.y) / 2.0f * m_Height, // Screen Y,
	//			ndcSpaceVertex.z,
	//			1.0f / viewSpaceVertex.z
	//		};
	//		screenSpaceVert.color = vert.color;
	//		screenSpaceVert.uv = vert.uv;

	//	}
	//}
#pragma endregion
#pragma region Week08


	for(Mesh& mesh : meshes)
	{
		// Calculate WorldViewProjectionmatrix for every mesh	
		Matrix worldViewProjectionMatrix = mesh.worldMatrix * (m_Camera.viewMatrix * m_Camera.projectionMatrix);

		mesh.vertices_out.clear();
		mesh.vertices_out.reserve(mesh.vertices.size());
		for(const Vertex& vert : mesh.vertices)
		{
			// World to camera (view space)
			Vector4 newPosition = worldViewProjectionMatrix.TransformPoint({ vert.position, 1.0f });

			// Perspective divide 
			newPosition.x /= newPosition.w;
			newPosition.y /= newPosition.w;
			newPosition.z /= newPosition.w;
			//newPosition.w = newPosition.w;

			// Our coords are now in NDC space

			// Multiply the normals and tangents with the worldmatrix to convert them to worldspace
			 //We only want to rotate them, so use transformvector, and normalize after
			const Vector3 newNormal = mesh.worldMatrix.TransformVector(vert.normal).Normalized();
			const Vector3 newTangent = mesh.worldMatrix.TransformVector(vert.tangent).Normalized();

			// Calculate vert world position
			const Vector3 vertPosition{ mesh.worldMatrix.TransformPoint(vert.position) };

			// Store the new position in the vertices out as Vertex out, because this one has a position 4 / vector4
			Vertex_Out& outVert = mesh.vertices_out.emplace_back(Vertex_Out{});
			outVert.position = newPosition;
			outVert.color = vert.color;
			outVert.uv = vert.uv;
			outVert.normal = newNormal;
			outVert.tangent = newTangent;
			outVert.viewDirection = { m_Camera.origin - vertPosition };
		}
	}

#pragma endregion
}




#ifdef Week06
void dae::Renderer::Render_W06_P1()
{
	// Triangle in NDC Space (Normalized Device Coords)
	std::vector<Vector3> vertices_ndc
	{
		{ 0.0f, 0.5f, 1.f },
		{ 0.5f, -0.5f, 1.f },
		{ -0.5f, -0.5f, 1.f }
	};


	// Convert the points of the triangle to screen space
	// Loop over every pixel (x & y)
	// Check if pixel is in the triangle (screen space)

	std::vector<Vector2> vertices_screen;
	for(const Vector3& vert : vertices_ndc)
	{
		// Screenspace has left top as 0 0, and bottom right at screenwidth screenheight, while normalized device space, starts at left bottom as -1, -1
		// and goes to top right 1, 1.
		// Screenspace vertexX = (VertexX + 1) / 2 * screenWidth
		// Screenspace vertexY = (1 - vertexY) / 2 * screenHeight


		vertices_screen.emplace_back(Vector2{
			(vert.x + 1) / 2.0f * m_Width , // Screen X
			(1 - vert.y) / 2.0f * m_Height
			// Screen Y
			});
	}


	for(int py = 0; py < m_Height; ++py)
	{
		for(int px = 0; px < m_Width; ++px)
		{
			Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

			// Define the edges of the screen triangle
			const Vector2 edgeA{ vertices_screen[0],-vertices_screen[1] }; // AB
			const Vector2 edgeB{ vertices_screen[1], vertices_screen[2] }; // BC
			const Vector2 edgeC{ vertices_screen[2], vertices_screen[0] }; // CA

			//const Vector2 triangleCenter{ (vertices_screen[0] + vertices_screen[1] + vertices_screen[2]) / 3.0f };

			bool isInside = true;
			// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)
			isInside &= Vector2::Cross(edgeA, Vector2{ vertices_screen[0], pixel }) >= 0.0f;
			isInside &= Vector2::Cross(edgeB, Vector2{ vertices_screen[1], pixel }) >= 0.0f;
			isInside &= Vector2::Cross(edgeC, Vector2{ vertices_screen[2], pixel }) >= 0.0f;

			if(isInside)
			{
				//		ColorRGB finalColor{ gradient, gradient, gradient };

				//Update Color in Buffer
				ColorRGB finalColor{ 1.0f, 1.0f, 1.0f };
				finalColor.MaxToOne();
				m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
					static_cast<uint8_t>(finalColor.r * 255),
					static_cast<uint8_t>(finalColor.g * 255),
					static_cast<uint8_t>(finalColor.b * 255));

			}
		}
	}
}

void dae::Renderer::Render_W06_P2()
{
	std::vector<Vertex> vertices_world{
		{{0.f, 2.f, 0.f}},
		{{1.f, 0.f, 0.f}},
		{{-1.f, 0.f, 0.f}}
	};

	std::vector<Vertex> vertices_screen{};

	VertexTransformationFunction(vertices_world, vertices_screen);

	// Load the texture


	for(int py = 0; py < m_Height; ++py)
	{
		for(int px = 0; px < m_Width; ++px)
		{
			Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

			// Vertices of the Triangle
			Vector2 A{ vertices_screen[0].position.x,  vertices_screen[0].position.y };
			Vector2 B{ vertices_screen[1].position.x,  vertices_screen[1].position.y };
			Vector2 C{ vertices_screen[2].position.x,  vertices_screen[2].position.y };


			// Define the edges of the screen triangle
			const Vector2 edgeA{ A, B };
			const Vector2 edgeB{ B, C };
			const Vector2 edgeC{ C, A };

			//const Vector2 triangleCenter{ (vertices_screen[0] + vertices_screen[1] + vertices_screen[2]) / 3.0f };
			bool isInside = true;

			// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)
			isInside &= Vector2::Cross(edgeA, Vector2{ A, pixel }) >= 0.0f;
			isInside &= Vector2::Cross(edgeB, Vector2{ B, pixel }) >= 0.0f;
			isInside &= Vector2::Cross(edgeC, Vector2{ C, pixel }) >= 0.0f;

			ColorRGB finalColor{ 0.0f, 0.0f, 0.0f };
			if(isInside)
			{
				// ColorRGB finalColor{ gradient, gradient, gradient };
				//Update Color in Buffer
				finalColor = { 1.0f, 1.0f, 1.0f };

			}

			finalColor.MaxToOne();
			m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}



}

void dae::Renderer::Render_W06_P3()
{
	std::vector<Vertex> vertices_world{
		{{0.f, 4.f, 2.f}, ColorRGB(colors::Red)},
		{{3.f, -2.f, 2.f},  ColorRGB(colors::Green)},
		{{-3.f, -2.f, 2.f},  ColorRGB(colors::Blue)}
	};

	std::vector<Vertex> vertices_screen{};

	VertexTransformationFunction(vertices_world, vertices_screen);


	for(int py = 0; py < m_Height; ++py)
	{
		for(int px = 0; px < m_Width; ++px)
		{
			Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

			// Vertices of the Triangle
			Vector2 A{ vertices_screen[0].position.x,  vertices_screen[0].position.y };
			Vector2 B{ vertices_screen[1].position.x,  vertices_screen[1].position.y };
			Vector2 C{ vertices_screen[2].position.x,  vertices_screen[2].position.y };


			// Define the edges of the screen triangle
			const Vector2 edgeA{ A, B };
			const Vector2 edgeB{ B, C };
			const Vector2 edgeC{ C, A };

			//const Vector2 triangleCenter{ (vertices_screen[0] + vertices_screen[1] + vertices_screen[2]) / 3.0f };

			// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)

			const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A, pixel }) };
			const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B, pixel }) };
			const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C, pixel }) };
			const float triangleArea = Vector2::Cross(edgeA, -edgeC);

			bool isInside = true;
			isInside &= signedAreaParallelogramAB >= 0.0f;
			isInside &= signedAreaParallelogramBC >= 0.0f;
			isInside &= signedAreaParallelogramCA >= 0.0f;



			ColorRGB finalColor{ 0.0f, 0.0f, 0.0f };
			if(isInside)
			{
				const float weightA{ signedAreaParallelogramAB / triangleArea };
				const float weightB{ signedAreaParallelogramBC / triangleArea };
				const float weightC{ signedAreaParallelogramCA / triangleArea };

				// Check if total weight is +/- 1.0f;
				assert((weightA + weightB + weightC) > 0.99f);
				assert((weightA + weightB + weightC) < 1.01f);



				ColorRGB interpolatedColor{ vertices_screen[2].color * weightA + vertices_screen[0].color * weightB + vertices_screen[1].color * weightC };

				// ColorRGB finalColor{ gradient, gradient, gradient };
				//Update Color in Buffer
				finalColor = { interpolatedColor };

			}

			finalColor.MaxToOne();
			m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

}

void dae::Renderer::Render_W06_P4()
{
	ClearBackBuffer();
	std::fill_n(m_pDepthBufferPixels, m_Width * m_Height, FLT_MAX);
	std::vector<Vertex> vertices_world{
		   {{0.f, 2.f, 0.f}, ColorRGB(colors::Red)},
		   {{1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},
		   {{-1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},

		   {{ 0.f, 4.f, 2.f}, ColorRGB(colors::Red)},
		   {{ 3.f, -2.f, 2.f}, ColorRGB(colors::Green)},
		   {{ -3.f, -2.f, 2.f}, ColorRGB(colors::Blue)}
	};

	std::vector<Vertex> vertices_screen{};

	VertexTransformationFunction(vertices_world, vertices_screen);



	for(int triangleIndex = 0; triangleIndex < vertices_screen.size(); triangleIndex += 3)
	{
		for(int py = 0; py < m_Height; ++py)
		{
			for(int px = 0; px < m_Width; ++px)
			{
				Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

				// Vertices of the Triangle
				Vertex A{ vertices_screen[triangleIndex + 0] };
				Vertex B{ vertices_screen[triangleIndex + 1] };
				Vertex C{ vertices_screen[triangleIndex + 2] };


				// Define the edges of the screen triangle
				const Vector2 edgeA{ A.position.GetXY(), B.position.GetXY() };
				const Vector2 edgeB{ B.position.GetXY(), C.position.GetXY() };
				const Vector2 edgeC{ C.position.GetXY(), A.position.GetXY() };

				//const Vector2 triangleCenter{ (vertices_screen[0] + vertices_screen[1] + vertices_screen[2]) / 3.0f };

				// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)

				const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A.position.GetXY(), pixel }) };
				const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B.position.GetXY(), pixel }) };
				const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C.position.GetXY(), pixel }) };
				const float triangleArea = Vector2::Cross(edgeA, -edgeC);

				bool isInside = true;
				isInside &= signedAreaParallelogramAB >= 0.0f;
				isInside &= signedAreaParallelogramBC >= 0.0f;
				isInside &= signedAreaParallelogramCA >= 0.0f;

				if(isInside)
				{
					// Get the weights of each vertex
					const float weightA{ signedAreaParallelogramAB / triangleArea };
					const float weightB{ signedAreaParallelogramBC / triangleArea };
					const float weightC{ signedAreaParallelogramCA / triangleArea };

					// Check if total weight is +/- 1.0f;
					assert((weightA + weightB + weightC) > 0.99f);
					assert((weightA + weightB + weightC) < 1.01f);


					// Float get the interpolated depth value using the barycentric weights
					float currentDepth = (A.position.z * weightA) + (B.position.z * weightB) + (C.position.z * weightC);


					// Check the depth buffer
					if(currentDepth > m_pDepthBufferPixels[px + (py * m_Width)])
						continue;

					m_pDepthBufferPixels[px + (py * m_Width)] = currentDepth;

					const ColorRGB color{ C.color * weightA + A.color * weightB + B.color * weightC };

					// ColorRGB finalColor{ gradient, gradient, gradient };
					//Update Color in Buffer

					ColorRGB finalColor = { color };

					finalColor.MaxToOne();
					m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
						static_cast<uint8_t>(finalColor.r * 255),
						static_cast<uint8_t>(finalColor.g * 255),
						static_cast<uint8_t>(finalColor.b * 255));

				}
			}
		}

	}
}

void dae::Renderer::Render_W06_P5()
{
	ClearBackBuffer();
	std::fill_n(m_pDepthBufferPixels, m_Width * m_Height, FLT_MAX);
	std::vector<Vertex> vertices_world{
		   {{0.f, 2.f, 0.f}, ColorRGB(colors::Red)},
		   {{1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},
		   {{-1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},

		   {{ 0.f, 4.f, 2.f}, ColorRGB(colors::Red)},
		   {{ 3.f, -2.f, 2.f}, ColorRGB(colors::Green)},
		   {{ -3.f, -2.f, 2.f}, ColorRGB(colors::Blue)}
	};

	std::vector<Vertex> vertices_screen{};

	VertexTransformationFunction(vertices_world, vertices_screen);



	for(int triangleIndex = 0; triangleIndex < vertices_screen.size(); triangleIndex += 3)
	{
		// Vertices of the Triangle
		const Vertex A{ vertices_screen[triangleIndex + 0] };
		const Vertex B{ vertices_screen[triangleIndex + 1] };
		const Vertex C{ vertices_screen[triangleIndex + 2] };

		// Define the edges of the screen triangle
		const Vector2 edgeA{ A.position.GetXY(), B.position.GetXY() };
		const Vector2 edgeB{ B.position.GetXY(), C.position.GetXY() };
		const Vector2 edgeC{ C.position.GetXY(), A.position.GetXY() };

		// Get the bounding box of the triangle (min max)
		Vector2 bbMin;
		bbMin.x = std::min(A.position.x, std::min(B.position.x, C.position.x));
		bbMin.y = std::min(A.position.y, std::min(B.position.y, C.position.y));

		Vector2 bbMax;
		bbMax.x = std::max(A.position.x, std::max(B.position.x, C.position.x));
		bbMax.y = std::max(A.position.y, std::max(B.position.y, C.position.y));

		bbMin.x = Clamp(bbMin.x, 0.0f, float(m_Width));
		bbMin.y = Clamp(bbMin.y, 0.0f, float(m_Height));

		bbMax.x = Clamp(bbMax.x, 0.0f, float(m_Width));
		bbMax.y = Clamp(bbMax.y, 0.0f, float(m_Height));

		for(int py = int(bbMin.y); py < int(bbMax.y); ++py)
		{
			for(int px = int(bbMin.x); px < int(bbMax.x); ++px)
			{
				// Get the current pixel into a vector
				Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

				// Get the signed areas of every edge (no division by 2 because triangle area isn't either, and we are only interested in percentage)
				const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A.position.GetXY(), pixel }) };
				const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B.position.GetXY(), pixel }) };
				const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C.position.GetXY(), pixel }) };
				const float triangleArea = Vector2::Cross(edgeA, -edgeC);

				// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)
				bool isInside = true;
				isInside &= signedAreaParallelogramAB >= 0.0f;
				isInside &= signedAreaParallelogramBC >= 0.0f;
				isInside &= signedAreaParallelogramCA >= 0.0f;

				if(isInside)
				{
					// Get the weights of each vertex
					const float weightA{ signedAreaParallelogramBC / triangleArea };
					const float weightB{ signedAreaParallelogramCA / triangleArea };
					const float weightC{ signedAreaParallelogramAB / triangleArea };

					// Check if total weight is +/- 1.0f;
					assert((weightA + weightB + weightC) > 0.99f);
					assert((weightA + weightB + weightC) < 1.01f);


					// Float get the interpolated depth value using the barycentric weights
					float currentDepth = (A.position.z * weightA) + (B.position.z * weightB) + (C.position.z * weightC);


					// Check the depth buffer
					if(currentDepth > m_pDepthBufferPixels[px + (py * m_Width)])
						continue;

					m_pDepthBufferPixels[px + (py * m_Width)] = currentDepth;

					const ColorRGB color{ A.color * weightA + B.color * weightB + C.color * weightC };

					// ColorRGB finalColor{ gradient, gradient, gradient };
					//Update Color in Buffer

					ColorRGB finalColor = { color };

					finalColor.MaxToOne();
					m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
						static_cast<uint8_t>(finalColor.r * 255),
						static_cast<uint8_t>(finalColor.g * 255),
						static_cast<uint8_t>(finalColor.b * 255));

				}
			}
		}
	}
}
#endif

void dae::Renderer::Render_W07_P1()
{
	ClearBackBuffer();
	std::fill_n(m_pDepthBufferPixels, m_Width * m_Height, FLT_MAX);
	std::vector<Vertex> vertices_world{
		   {{0.f, 2.f, 0.f}, ColorRGB(colors::Red)},
		   {{1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},
		   {{-1.5f, -1.f, 0.f}, ColorRGB(colors::Red)},

		   {{ 0.f, 4.f, 2.f}, ColorRGB(colors::Red)},
		   {{ 3.f, -2.f, 2.f}, ColorRGB(colors::Green)},
		   {{ -3.f, -2.f, 2.f}, ColorRGB(colors::Blue)}
	};

	std::vector<Mesh> meshes
	{
		/*
		Mesh
		{
			{
				Vertex{{ -3,3,-2 }, {}, {0, 0}},
				Vertex{{ 0,3,-2 }, {}, {.5f, 0}},
				Vertex{{ 3,3,-2 }, {}, {1, 0}},
				Vertex{{ -3,0,-2 }, {}, {0, .5f}},
				Vertex{{ 0,0,-2 }, {}, {.5f, .5f}},
				Vertex{{ 3,0,-2 }, {}, {1, .5f}},
				Vertex{{ -3,-3,-2 }, {}, {0, 1}},
				Vertex{{ 0,-3,-2 }, {}, {.5f, 1}},
				Vertex{{ 3,-3,-2 }, {}, {1, 1}},
			},
			{
				3,0,4,1,5,2,
				2,6,
				6,3,7,4,8,5
			},
			PrimitiveTopology::TriangleStrip
		},
		*/
		Mesh
		{
			{
				Vertex{{ -3,3,-2 }, {}, {0, 0}},
				Vertex{{ 0,3,-2 }, {}, {.5f, 0}},
				Vertex{{ 3,3,-2 }, {}, {1, 0}},
				Vertex{{ -3,0,-2 }, {}, {0, .5f}},
				Vertex{{ 0,0,-2 }, {}, {.5f, .5f}},
				Vertex{{ 3,0,-2 }, {}, {1, .5f}},
				Vertex{{ -3,-3,-2 }, {}, {0, 1}},
				Vertex{{ 0,-3,-2 }, {}, {.5f, 1}},
				Vertex{{ 3,-3,-2 }, {}, {1, 1}},
			},
			{
				3,0,1,   1,4,3,   4,1,2,
				2,5,4,   6,3,4,   4,7,6,
				7,4,5,   5,8,7,
			},
			PrimitiveTopology::TriangleList
		}
	};


	VertexTransformationFunction(meshes);

	for(const Mesh& mesh : meshes)
	{
		//VertexTransformationFunction(mesh.vertices, mesh_screen.vertices);

		// If triangle strip, move only one position per itteration & inverse the direction on every odd loop
		int increment = 3;
		if(mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			increment = 1;

		for(int indiceIdx = 0; indiceIdx < mesh.indices.size() - 2; indiceIdx += increment)
		{
			// Get the vertices using the indice numbers
			uint32_t indiceA{ mesh.indices[indiceIdx] };
			uint32_t indiceB{ mesh.indices[indiceIdx + 1] };
			uint32_t indiceC{ mesh.indices[indiceIdx + 2] };

			Vertex_Out A{ mesh.vertices_out[indiceA] };
			Vertex_Out B{ mesh.vertices_out[indiceB] };
			Vertex_Out C{ mesh.vertices_out[indiceC] };

			// If triangle strip, move only one position per itteration & inverse the direction on every odd loop

			if(mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			{
				// Check if least significant bit is 1 (odd number)
				if((indiceIdx & 1) == 1)
					std::swap(B, C);

				// Check if any vertices of the triangle are the same (and thus the triangle has 0 area / should not be rendered)
				if(indiceA == indiceB)
					continue;

				if(indiceB == indiceC)
					continue;

				if(indiceC == indiceA)
					continue;
			}

			// Define the edges of the screen triangle
			const Vector2 edgeA{ A.position.GetXY(), B.position.GetXY() };
			const Vector2 edgeB{ B.position.GetXY(), C.position.GetXY() };
			const Vector2 edgeC{ C.position.GetXY(), A.position.GetXY() };

			// Get the bounding box of the triangle (min max)
			Vector2 bbMin;
			bbMin.x = std::min(A.position.x, std::min(B.position.x, C.position.x));
			bbMin.y = std::min(A.position.y, std::min(B.position.y, C.position.y));

			Vector2 bbMax;
			bbMax.x = std::max(A.position.x, std::max(B.position.x, C.position.x));
			bbMax.y = std::max(A.position.y, std::max(B.position.y, C.position.y));

			bbMin.x = Clamp(bbMin.x, 0.0f, float(m_Width));
			bbMin.y = Clamp(bbMin.y, 0.0f, float(m_Height));

			bbMax.x = Clamp(bbMax.x, 0.0f, float(m_Width));
			bbMax.y = Clamp(bbMax.y, 0.0f, float(m_Height));



			for(int py = int(bbMin.y); py < int(bbMax.y); ++py)
			{
				for(int px = int(bbMin.x); px < int(bbMax.x); ++px)
				{
					// Get the current pixel into a vector
					Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

					// Get the signed areas of every edge (no division by 2 because triangle area isn't either, and we are only interested in percentage)
					const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A.position.GetXY(), pixel }) };
					const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B.position.GetXY(), pixel }) };
					const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C.position.GetXY(), pixel }) };
					const float triangleArea = Vector2::Cross(edgeA, -edgeC);

					// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)
					bool isInside = true;
					isInside &= signedAreaParallelogramAB >= 0.0f;
					isInside &= signedAreaParallelogramBC >= 0.0f;
					isInside &= signedAreaParallelogramCA >= 0.0f;

					if(isInside)
					{
						// Get the weights of each vertex
						const float weightA{ signedAreaParallelogramBC / triangleArea };
						const float weightB{ signedAreaParallelogramCA / triangleArea };
						const float weightC{ signedAreaParallelogramAB / triangleArea };

						// Check if total weight is +/- 1.0f;
						assert((weightA + weightB + weightC) > 0.99f);
						assert((weightA + weightB + weightC) < 1.01f);

						// Float get the interpolated depth value using the barycentric weights
						//float currentDepth = (A.position.z * weightA) + (B.position.z * weightB) + (C.position.z * weightC);
						const float zInterpolated = 1.0f / ((A.position.w * weightA) + (B.position.w * weightB) + (C.position.w * weightC));

						// Check the depth buffer
						if(zInterpolated > m_pDepthBufferPixels[px + (py * m_Width)])
							continue;

						m_pDepthBufferPixels[px + (py * m_Width)] = zInterpolated;

						//const ColorRGB color{ A.color * weightA + B.color * weightB + C.color * weightC };

						// ColorRGB finalColor{ gradient, gradient, gradient };
						//Update Color in Buffer

						Vector2 uvInterpolated = {
							(A.uv / A.position.z) * weightA +
							(B.uv / B.position.z) * weightB +
							(C.uv / C.position.z) * weightC
						};

						uvInterpolated *= zInterpolated;


						ColorRGB finalColor = { m_pTexture->Sample(uvInterpolated) };

						finalColor.MaxToOne();
						m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
							static_cast<uint8_t>(finalColor.r * 255),
							static_cast<uint8_t>(finalColor.g * 255),
							static_cast<uint8_t>(finalColor.b * 255));

					}
				}
			}
		}
	}
}

void dae::Renderer::Render_W08_P1()
{
	ClearBackBuffer();
	std::fill_n(m_pDepthBufferPixels, m_Width * m_Height, FLT_MAX);

	VertexTransformationFunction(m_Meshes);

	for(const Mesh& mesh : m_Meshes)
	{
		//VertexTransformationFunction(mesh.vertices, mesh_screen.vertices);

		// If triangle strip, move only one position per itteration & inverse the direction on every odd loop
		int increment = 3;
		if(mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			increment = 1;

		for(int indiceIdx = 0; indiceIdx < mesh.indices.size() - 2; indiceIdx += increment)
		{
			// Get the vertices using the indice numbers
			uint32_t indiceA{ mesh.indices[indiceIdx] };
			uint32_t indiceB{ mesh.indices[indiceIdx + 1] };
			uint32_t indiceC{ mesh.indices[indiceIdx + 2] };

			Vertex_Out A{ mesh.vertices_out[indiceA] };
			Vertex_Out B{ mesh.vertices_out[indiceB] };
			Vertex_Out C{ mesh.vertices_out[indiceC] };

			// If triangle strip, move only one position per itteration & inverse the direction on every odd loop

			if(mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			{
				// Check if least significant bit is 1 (odd number)
				if((indiceIdx & 1) == 1)
					std::swap(B, C);

				// Check if any vertices of the triangle are the same (and thus the triangle has 0 area / should not be rendered)
				if(indiceA == indiceB)
					continue;

				if(indiceB == indiceC)
					continue;

				if(indiceC == indiceA)
					continue;

			}


			// Do frustum culling
			if(A.position.z < 0.0f || A.position.z > 1.0f)
				continue;
			if(B.position.z < 0.0f || B.position.z > 1.0f)
				continue;
			if(C.position.z < 0.0f || C.position.z > 1.0f)
				continue;

			if(A.position.x < -1.0f || A.position.x > 1.0f)
				//continue;
				if(B.position.x < -1.0f || B.position.x > 1.0f)
					//continue;
					if(C.position.x < -1.0f || C.position.x > 1.0f)
						continue;

			if(A.position.y < -1.0f || A.position.y > 1.0f)
				//continue;
				if(B.position.y < -1.0f || B.position.y > 1.0f)
					//continue;
					if(C.position.y < -1.0f || C.position.y > 1.0f)
						continue;


			// Convert from NDC to ScreenSpace
			A.position.x = (A.position.x + 1) / 2.0f * m_Width; // Screen X
			A.position.y = (1 - A.position.y) / 2.0f * m_Height; // Screen Y,
			B.position.x = (B.position.x + 1) / 2.0f * m_Width; // Screen X
			B.position.y = (1 - B.position.y) / 2.0f * m_Height; // Screen Y,
			C.position.x = (C.position.x + 1) / 2.0f * m_Width; // Screen X
			C.position.y = (1 - C.position.y) / 2.0f * m_Height; // Screen Y,

			// Define the edges of the screen triangle
			const Vector2 edgeA{ A.position.GetXY(), B.position.GetXY() };
			const Vector2 edgeB{ B.position.GetXY(), C.position.GetXY() };
			const Vector2 edgeC{ C.position.GetXY(), A.position.GetXY() };


			// Get the bounding box of the triangle (min max)
			Vector2 bbMin;
			bbMin.x = std::min(A.position.x, std::min(B.position.x, C.position.x));
			bbMin.y = std::min(A.position.y, std::min(B.position.y, C.position.y));

			Vector2 bbMax;
			bbMax.x = std::max(A.position.x, std::max(B.position.x, C.position.x));
			bbMax.y = std::max(A.position.y, std::max(B.position.y, C.position.y));

			bbMin.x = Clamp(bbMin.x, 0.0f, float(m_Width));
			bbMin.y = Clamp(bbMin.y, 0.0f, float(m_Height));

			bbMax.x = Clamp(bbMax.x, 0.0f, float(m_Width));
			bbMax.y = Clamp(bbMax.y, 0.0f, float(m_Height));



			for(int py = int(bbMin.y); py < int(ceil(bbMax.y)); ++py)
			{
				for(int px = int(bbMin.x); px < int(ceil(bbMax.x)); ++px)
				{
					// Get the current pixel into a vector
					Vector2 pixel{ float(px) + 0.5f, float(py) + 0.5f };  // Define pixel as 2D point (take center of the pixel)

					// Get the signed areas of every edge (no division by 2 because triangle area isn't either, and we are only interested in percentage)
					const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A.position.GetXY(), pixel }) };
					const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B.position.GetXY(), pixel }) };
					const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C.position.GetXY(), pixel }) };

					// isInside will turn false if any of the below 3 caclulations returns a negative number (true &= true -> true while true &= false -> false)
					bool isInside = true;
					isInside &= signedAreaParallelogramAB >= 0.0f;
					isInside &= signedAreaParallelogramBC >= 0.0f;
					isInside &= signedAreaParallelogramCA >= 0.0f;

					if(isInside)
					{
						// Perform clipping
						//if (A.position.x < -1.0f || A.position.x > 1.0f)
						//	continue;

						//if (A.position.y < -1.0f || A.position.y > 1.0f)
						//	continue;


						// Get the weights of each vertex
						const float triangleArea = Vector2::Cross(edgeA, -edgeC);
						const float weightA{ signedAreaParallelogramBC / triangleArea };
						const float weightB{ signedAreaParallelogramCA / triangleArea };
						const float weightC{ signedAreaParallelogramAB / triangleArea };

						// Check if total weight is +/- 1.0f;
						assert((weightA + weightB + weightC) > 0.99f);
						assert((weightA + weightB + weightC) < 1.01f);

						// Get the interpolated Z buffer value
						float zBuffer = 1.0f /
							((1.0f / A.position.z) * weightA + (1.0f / B.position.z) * weightB + (1.0f / C.position.z) * weightC);

						// Check the depth buffer
						if(zBuffer > m_pDepthBufferPixels[px + (py * m_Width)])
							continue;

						m_pDepthBufferPixels[px + (py * m_Width)] = zBuffer;

						float wInterpolated = 1.0f /
							((1.0f / A.position.w) * weightA + (1.0f / B.position.w) * weightB + (1.0f / C.position.w) * weightC);

						// Get the interpolated UV
						Vector2 uvInterpolated{
							(A.uv / A.position.w) * weightA +
							(B.uv / B.position.w) * weightB +
							(C.uv / C.position.w) * weightC
						};
						uvInterpolated *= wInterpolated;

						// Get the interpolated color
						ColorRGB colorInterpolated{
							(A.color / A.position.w) * weightA +
							(B.color / B.position.w) * weightB +
							(C.color / C.position.w) * weightC
						};
						colorInterpolated *= wInterpolated;

						// Get the interpolated normal
						Vector3 normalInterpolated{
							(A.normal / A.position.w) * weightA +
							(B.normal / B.position.w) * weightB +
							(C.normal / C.position.w) * weightC
						};
						normalInterpolated *= wInterpolated;
						normalInterpolated.Normalize();

						// Get the interpolated tangent
						Vector3 tangentInterpolated{
							(A.tangent / A.position.w) * weightA +
							(B.tangent / B.position.w) * weightB +
							(C.tangent / C.position.w) * weightC
						};
						tangentInterpolated *= wInterpolated;
						tangentInterpolated.Normalize();

						// Get the interpolated viewdirection
						Vector3 viewDirectionInterpolated{
							(A.viewDirection / A.position.w) * weightA +
							(B.viewDirection / B.position.w) * weightB +
							(C.viewDirection / C.position.w) * weightC
						};
						viewDirectionInterpolated *= wInterpolated;
						viewDirectionInterpolated.Normalize();


						Vertex_Out vertexOut{};
						vertexOut.position = Vector4{ pixel.x, pixel.y, zBuffer, wInterpolated };
						vertexOut.color = colorInterpolated;
						vertexOut.uv = uvInterpolated;
						vertexOut.normal = normalInterpolated;
						vertexOut.tangent = tangentInterpolated;
						vertexOut.viewDirection = viewDirectionInterpolated;

						//PixelShading(vertexOut);

						ColorRGB finalColor{};
						switch(m_CurrentRenderMode)
						{
							case dae::Renderer::RenderMode::FinalColor:
								finalColor = PixelShading(vertexOut);
								//finalColor = { m_pTexture->Sample(uvInterpolated) };
								break;
							case dae::Renderer::RenderMode::DepthBuffer:
							{
								const float remapMin{ 0.985f };
								const float remapMax{ 1.0f };

								float depthColor = (Clamp(zBuffer, remapMin, remapMax) - remapMin) * (1.0f / (remapMax - remapMin));
								//depthColor = 1.0f - depthColor;  // Invert white and black



								finalColor = { depthColor, depthColor, depthColor };

							}
							break;
							default:
								break;
						}

						finalColor.MaxToOne();
						m_pBackBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBackBuffer->format,
							static_cast<uint8_t>(finalColor.r * 255),
							static_cast<uint8_t>(finalColor.g * 255),
							static_cast<uint8_t>(finalColor.b * 255));

					}
				}
			}
		}
	}
}



ColorRGB dae::Renderer::PixelShading(const Vertex_Out& vert)
{
	const Vector3 lightDirection{ .577f, -.577f, .577f };
	const ColorRGB lightColor{ 1.0f, 1.0f, 1.0f };
	const float lightIntensity{ 7.0f };
	const ColorRGB lightRadiance{ lightColor * lightIntensity };
	const float specularGlossiness{ 25.0f };  // Shininess
	const ColorRGB ambientColor{ 0.025f, 0.025f, 0.025f };

	const ColorRGB diffuseColorSample{ m_pTexture->Sample(vert.uv) };
	const ColorRGB specularColorSample{ m_pSpecular->Sample(vert.uv) };
	const ColorRGB normalColorSample{ m_pNormal->Sample(vert.uv) };
	const ColorRGB glossinessColor{ m_pGloss->Sample(vert.uv) };


	//// Calculate tangent space axis
	const Vector3 binormal{ Vector3::Cross(vert.normal, vert.tangent) };
	const Matrix tangentSpaceAxis{ Matrix{ vert.tangent, binormal, vert.normal, {0,0,0} } };  // {} = 0 vector

	////Calculate normal in tangent space
	const Vector3 tangentNormal{ normalColorSample.r * 2.0f - 1.0f, normalColorSample.g * 2.0f - 1.0f, normalColorSample.b * 2.0f - 1.0f };
	const Vector3 normalInTangentSpace{ tangentSpaceAxis.TransformVector(tangentNormal.Normalized()).Normalized() };

	//// Select normal based on settings
	const Vector3 currentNormal{ m_NormalsEnabled ? normalInTangentSpace : vert.normal };

	//// Calculate observed area / lambert Cosine
	const float observedArea{ Vector3::Dot(currentNormal, -lightDirection) };

	if(observedArea < 0.0f)
		return { 0,0,0 };


	// Calculate lambert
	const ColorRGB lambertDiffuse{ (1.0f * diffuseColorSample) / PI };

	// Calculate phong
	const Vector3 reflect{ lightDirection - (2.0f * Vector3::Dot(currentNormal, lightDirection) * currentNormal) };
	const float RdotV{ std::max(0.0f, Vector3::Dot(reflect, vert.viewDirection)) };
	const ColorRGB phongSpecular{ specularColorSample * powf(RdotV, glossinessColor.r * specularGlossiness) }; // Glosinness map is greyscale, ro r g and b are the same

	switch(m_CurrentShadingMode)
	{
		case dae::Renderer::ShadingMode::Combined:
			return ((lightRadiance * lambertDiffuse) + phongSpecular + ambientColor) * observedArea;
			break;
		case dae::Renderer::ShadingMode::ObservedArea:
			return ColorRGB{ observedArea, observedArea, observedArea };
			break;
		case dae::Renderer::ShadingMode::Diffuse:
			return lightRadiance * lambertDiffuse * observedArea;
			break;
		case dae::Renderer::ShadingMode::Specular:
			return phongSpecular;
			break;
		default:
			return { 0.f, 0.f, 0.5f };
			break;
	}

}

void dae::Renderer::ToggleNormals()
{
	m_NormalsEnabled = !m_NormalsEnabled;
	if(m_NormalsEnabled)
	{
		std::cout << "Normal map\n";
	}
	else
	{
		std::cout << "Vert normals\n";
	}
}

void dae::Renderer::ToggleShadingMode()
{
	switch(m_CurrentShadingMode)
	{
		case dae::Renderer::ShadingMode::Combined:
			m_CurrentShadingMode = ShadingMode::ObservedArea;
			std::cout << "ShadingMode: ObservedArea\n";
			break;
		case dae::Renderer::ShadingMode::ObservedArea:
			m_CurrentShadingMode = ShadingMode::Diffuse;
			std::cout << "ShadingMode: Diffuse\n";
			break;
		case dae::Renderer::ShadingMode::Diffuse:
			m_CurrentShadingMode = ShadingMode::Specular;
			std::cout << "ShadingMode: Specular\n";
			break;
		case dae::Renderer::ShadingMode::Specular:
			m_CurrentShadingMode = ShadingMode::Combined;
			std::cout << "ShadingMode: Combined\n";
			break;
		default:
			assert(false); // Should never hit this
			break;
	}
}

void dae::Renderer::ClearBackBuffer()
{
	ColorRGB clearColor{ 100, 100, 100 };

	uint32_t hexColor = 0xFF000000 | (uint32_t)clearColor.b << 16 | (uint32_t)clearColor.g << 8 | (uint32_t)clearColor.r;
	SDL_FillRect(m_pBackBuffer, NULL, hexColor);

	//SDL_FillRect(m_pBackBuffer, &m_pBackBuffer->clip_rect, SDL_MapRGB(m_pBackBuffer->format, clearColor.r, clearColor.g, clearColor.b));

}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}

void dae::Renderer::ToggleRenderMode()
{
	int cntRenderModes = 2;
	m_CurrentRenderMode = RenderMode(((int)m_CurrentRenderMode + 1) % cntRenderModes);

}
