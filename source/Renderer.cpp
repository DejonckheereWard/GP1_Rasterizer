//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Texture.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window* pWindow) :
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
	m_Camera.Initialize(60.f, { .0f,.0f,-10.f });

	// Temp texture init

    m_pTexture = Texture::LoadFromFile("./Resources/uv_grid_2.png");
}

Renderer::~Renderer()
{
	delete[] m_pDepthBufferPixels;
	delete m_pTexture;
}

void Renderer::Update(Timer* pTimer)
{
	m_Camera.Update(pTimer);
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

	Render_W07_P1();


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

	for (const Vertex& vert : vertices_in)
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

	//vertices_out.clear();
	//vertices_out.reserve(meshes.size() * 3);

	for (Mesh& mesh : meshes)
	{
		for (const Vertex& vert : mesh.vertices)
		{
			// World to camera (view space)
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
			Vertex_Out& screenSpaceVert = mesh.vertices_out.emplace_back(Vertex_Out{});
			screenSpaceVert.position = {
				(ndcSpaceVertex.x + 1) / 2.0f * m_Width , // Screen X
				(1 - ndcSpaceVertex.y) / 2.0f * m_Height, // Screen Y,
				ndcSpaceVertex.z,
				1.0f / viewSpaceVertex.z
			};
			screenSpaceVert.color = vert.color;
			screenSpaceVert.uv = vert.uv;

		}
	}
}


void dae::Renderer::ClearBackBuffer()
{
	ColorRGB clearColor{ 100, 100, 100 };
	uint32_t hexColor = 0xFF000000 | (uint32_t)clearColor.b << 16 | (uint32_t)clearColor.g << 8 | (uint32_t)clearColor.r;
	SDL_FillRect(m_pBackBuffer, NULL, hexColor);
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
	for (const Vector3& vert : vertices_ndc)
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


	for (int py = 0; py < m_Height; ++py)
	{
		for (int px = 0; px < m_Width; ++px)
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

			if (isInside)
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


	for (int py = 0; py < m_Height; ++py)
	{
		for (int px = 0; px < m_Width; ++px)
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
			if (isInside)
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


	for (int py = 0; py < m_Height; ++py)
	{
		for (int px = 0; px < m_Width; ++px)
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
			if (isInside)
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



	for (int triangleIndex = 0; triangleIndex < vertices_screen.size(); triangleIndex += 3)
	{
		for (int py = 0; py < m_Height; ++py)
		{
			for (int px = 0; px < m_Width; ++px)
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

				const float signedAreaParallelogramAB{ Vector2::Cross(edgeA, Vector2{ A.position.GetXY(), pixel}) };
				const float signedAreaParallelogramBC{ Vector2::Cross(edgeB, Vector2{ B.position.GetXY(), pixel }) };
				const float signedAreaParallelogramCA{ Vector2::Cross(edgeC, Vector2{ C.position.GetXY(), pixel }) };
				const float triangleArea = Vector2::Cross(edgeA, -edgeC);

				bool isInside = true;
				isInside &= signedAreaParallelogramAB >= 0.0f;
				isInside &= signedAreaParallelogramBC >= 0.0f;
				isInside &= signedAreaParallelogramCA >= 0.0f;

				if (isInside)
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
					if (currentDepth > m_pDepthBufferPixels[px + (py * m_Width)])
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



	for (int triangleIndex = 0; triangleIndex < vertices_screen.size(); triangleIndex += 3)
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

		for (int py = int(bbMin.y); py < int(bbMax.y); ++py)
		{
			for (int px = int(bbMin.x); px < int(bbMax.x); ++px)
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

				if (isInside)
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
					if (currentDepth > m_pDepthBufferPixels[px + (py * m_Width)])
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

	for (const Mesh& mesh : meshes)
	{
		//VertexTransformationFunction(mesh.vertices, mesh_screen.vertices);

		// If triangle strip, move only one position per itteration & inverse the direction on every odd loop
		int increment = 3;
		if (mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			increment = 1;

		for (int indiceIdx = 0; indiceIdx < mesh.indices.size() - 2; indiceIdx += increment)
		{
			// Get the vertices using the indice numbers
			uint32_t indiceA{ mesh.indices[indiceIdx] };
			uint32_t indiceB{ mesh.indices[indiceIdx + 1] };
			uint32_t indiceC{ mesh.indices[indiceIdx + 2] };

			Vertex_Out A{ mesh.vertices_out[indiceA] };
			Vertex_Out B{ mesh.vertices_out[indiceB] };
			Vertex_Out C{ mesh.vertices_out[indiceC] };

			// If triangle strip, move only one position per itteration & inverse the direction on every odd loop

			if (mesh.primitiveTopology == PrimitiveTopology::TriangleStrip)
			{
				// Check if least significant bit is 1 (odd number)
				if ((indiceIdx & 1) == 1)
					std::swap(B, C);

				// Check if any vertices of the triangle are the same (and thus the triangle has 0 area / should not be rendered)
				if (indiceA == indiceB)
					continue;

				if (indiceB == indiceC)
					continue;

				if (indiceC == indiceA)
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

			for (int py = int(bbMin.y); py < int(bbMax.y); ++py)
			{
				for (int px = int(bbMin.x); px < int(bbMax.x); ++px)
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

					if (isInside)
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
						if (zInterpolated > m_pDepthBufferPixels[px + (py * m_Width)])
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

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBackBuffer, "Rasterizer_ColorBuffer.bmp");
}
