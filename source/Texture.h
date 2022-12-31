#pragma once
#include <SDL_surface.h>
#include <string>
#include "ColorRGB.h"

namespace dae
{
	struct Vector2;

	enum class UVMode
	{
		Wrap,
		Clamp,
		Mirror,
		Border
	};

	class Texture
	{
	public:
		~Texture();

		static Texture* LoadFromFile(const std::string& path);
		ColorRGB Sample(const Vector2& uv, UVMode uvMode = UVMode::Wrap) const;

	private:
		Texture(SDL_Surface* pSurface);

		SDL_Surface* m_pSurface{ nullptr };
		uint32_t* m_pSurfacePixels{ nullptr };
	};
}