#include "Texture.h"
#include "Vector2.h"
#include <SDL_image.h>
#include <cassert>

namespace dae
{
	Texture::Texture(SDL_Surface* pSurface) :
		m_pSurface{ pSurface },
		m_pSurfacePixels{ (uint32_t*)pSurface->pixels }
	{
	}

	Texture::~Texture()
	{
		if (m_pSurface)
		{
			SDL_FreeSurface(m_pSurface);
			m_pSurface = nullptr;
		}
	}


	// Static function
	Texture* Texture::LoadFromFile(const std::string& path)
	{
		//TODO
		//Load SDL_Surface using IMG_LOAD
		//Create & Return a new Texture Object (using SDL_Surface)
		SDL_Surface* newSurface = IMG_Load(path.c_str());
		assert(newSurface != nullptr);

		return new Texture(newSurface);
	}

	ColorRGB Texture::Sample(const Vector2& uv, UVMode uvMode) const
	{

		int x = 0;
		int y = 0;

		switch (uvMode)
		{
			
		case dae::UVMode::Wrap:
		{
			float uvX = uv.x;
			float uvY = uv.y;
			
			if (uvX < 0)
				uvX += abs(int(uvX)) + 1;
			if (uvY < 0)
				uvY += abs(int(uvY)) + 1;

			x = int(uvX * m_pSurface->w) % m_pSurface->w;
			y = int(uvY * m_pSurface->h) % m_pSurface->h;
			break;
		}
		
		case dae::UVMode::Clamp:
			x = int(uv.x * m_pSurface->w);
			y = int(uv.y * m_pSurface->h);
			x = Clamp(x, 0, m_pSurface->w);
			y = Clamp(y, 0, m_pSurface->h);
			break;
			
		case dae::UVMode::Mirror:
			x = int(uv.x * m_pSurface->w);
			y = int(uv.y * m_pSurface->h);
			if (x % 2 == 0)
				x = x % m_pSurface->w;
			else
				x = m_pSurface->w - (x % m_pSurface->w);
			break;
			
		default:
			assert(false && "Shouldn't ever hit this in the switch");
			break;
			
		}

		// pixel color is in 0-255 ranges  0xFF FF FF FF -> ALPHA, BLUE, GREEN, RED
		const uint32_t pixelColor = m_pSurfacePixels[(y * m_pSurface->w) + x];

		ColorRGB color{};
		color.r = float((pixelColor >> 0) & 0xFF);  // 0 shift because RED is least significnat, 0xFF because we want to mask out the other colors (only take last byte)
		color.g = float((pixelColor >> 8) & 0xFF);
		color.b = float((pixelColor >> 16) & 0xFF);
		//color.a = (pixelColor >> 24) & 0xFF; // No A component apparently 

		color /= 255.0f;  // "Normalize" from 0->1

		return color;
	}
}