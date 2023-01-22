#pragma once

#include <cstdint>
#include <vector>

#include "Camera.h"
#include "DataTypes.h"

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Texture;
	struct Mesh;
	struct Vertex;
	class Timer;
	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer();

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Update(Timer* pTimer);
		void Render();

		bool SaveBufferToImage() const;
		void ToggleRenderMode();
		void ToggleRotation() { m_RotationEnabled = !m_RotationEnabled; }
		void ToggleNormals();
		void ToggleShadingMode();

	private:
		enum class RenderMode
		{
			FinalColor,
			DepthBuffer
		};

		enum class ShadingMode
		{
			Combined,
			ObservedArea,
			Diffuse,		// Include ObservedArea
			Specular		// Include ObservedArea
		};

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pFrontBuffer{ nullptr };
		SDL_Surface* m_pBackBuffer{ nullptr };
		uint32_t* m_pBackBufferPixels{};

		Texture* m_pTexture{ nullptr };
		Texture* m_pNormal{ nullptr };
		Texture* m_pSpecular{ nullptr };
		Texture* m_pGloss{ nullptr };

		float* m_pDepthBufferPixels{};

		Camera m_Camera{};
		std::vector<Mesh> m_Meshes{};

		int m_Width{};
		int m_Height{};

		RenderMode m_CurrentRenderMode{ RenderMode::FinalColor };
		ShadingMode m_CurrentShadingMode{ ShadingMode::Combined };
		bool m_RotationEnabled{ true };
		bool m_NormalsEnabled{ true };
		float m_CurrentMeshRotation{ 0.0f };

		//Function that transforms the vertices from the mesh from World space to Screen space
		void VertexTransformationFunction(const std::vector<Vertex>& vertices_in, std::vector<Vertex>& vertices_out) const; //W1 Version
		void VertexTransformationFunction(std::vector<Mesh>& meshes) const; //W1 Version
		void ClearBackBuffer();
		ColorRGB PixelShading(const Vertex_Out& vert);

#ifdef Old
		void Render_W06_P1();  // Rasterizer stage only
		void Render_W06_P2();  // Projection stage only
		void Render_W06_P3();  // Barycentric coords
		void Render_W06_P4();  // Depth buffer
		void Render_W06_P5();  // Bounding box optimization
		void Render_W07_P1();
#endif


		void Rasterize();


	};
}
