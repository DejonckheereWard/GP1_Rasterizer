#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"



namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{
		}


		Vector3 origin{};
		float fovAngle{ 90.f };
		float fov{ tanf((fovAngle * TO_RADIANS) / 2.f) };
		const float far{ 100.f };
		const float near{ 0.1f };
		float aspectRatio{ 1.f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		const float movementSpeed{ 15.0f };
		const float rotationSpeed{ 30.0f };
		const float keyboardRotationSpeed{ 80.0f };

		float totalPitch{};
		float totalYaw{};

		Matrix invViewMatrix{};
		Matrix viewMatrix{};

		Matrix projectionMatrix{};

		bool UpdateViewMatrix{ true };

		void Initialize(float _fovAngle = 90.f, Vector3 _origin = { 0.f,0.f,0.f }, float _aspectRatio = 1.0f)
		{
			fovAngle = _fovAngle;
			fov = tanf((fovAngle * TO_RADIANS) / 2.f);
			aspectRatio = _aspectRatio;
			origin = _origin;

			CalculateProjectionMatrix();
		}

		void CalculateViewMatrix()
		{
			if (!UpdateViewMatrix)
				return;
			//TODO W1
			//ONB => invViewMatrix
			//Inverse(ONB) => ViewMatrix

			//invViewMatrix => Matrix::CreateLookAtLH(...) [not implemented yet]
			//DirectX Implementation => https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dxmatrixlookatlh

			invViewMatrix = Matrix::CreateLookAtLH(origin, forward, up);
			viewMatrix = invViewMatrix.Inverse();
			UpdateViewMatrix = false;


		}

		void CalculateProjectionMatrix()
		{
			// W
			projectionMatrix = Matrix::CreatePerspectiveFovLH(fov, aspectRatio, near, far);
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Camera Update Logic
			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			bool hasMoved{ false };

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			// Keyboard movement of the camera
			if (pKeyboardState[SDL_SCANCODE_W])
			{
				origin += forward * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_S])
			{
				origin -= forward * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_D])
			{
				origin += right * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_A])
			{
				origin -= right * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_SPACE])
			{
				origin += up * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_LSHIFT])
			{
				origin -= up * movementSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_UP])
			{
				totalPitch += keyboardRotationSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_DOWN])
			{
				totalPitch -= keyboardRotationSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_LEFT])
			{
				totalYaw -= keyboardRotationSpeed * deltaTime;
				hasMoved = true;
			}
			if (pKeyboardState[SDL_SCANCODE_RIGHT])
			{
				totalYaw += keyboardRotationSpeed * deltaTime;
				hasMoved = true;
			}

			// Mouse movements / rotation of the camera
			if ((mouseState & SDL_BUTTON(SDL_BUTTON_LEFT)) && mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT))
			{
				// mouseX yaw left & right, mouse Y moves forwards & backwards
				const float upwards = -mouseY * movementSpeed * deltaTime;
				origin += up * upwards;
				hasMoved = true;
			}
			else if (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT))
			{
				// mouseX yaw left & right, mouse Y moves forwards & backwards
				const float forwards = -mouseY * deltaTime;
				const float yaw = mouseX * deltaTime;

				origin += forward * forwards;
				totalYaw += yaw;
				hasMoved = true;
			}
			else if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT))
			{
				// Look around the current origin
				const float pitch = -mouseY * rotationSpeed * deltaTime;
				const float yaw = mouseX * rotationSpeed * deltaTime;

				totalPitch += pitch;
				totalYaw += yaw;
				hasMoved = true;
			}

			if (hasMoved)
			{
				totalPitch = Clamp(totalPitch, -89.9f, 89.9f);
				if (totalYaw > 360.0f)
					totalYaw -= 360.0f;
				else if (totalYaw < 0.0f)
					totalYaw += 360.0f;

				const Matrix finalRotation = Matrix::CreateRotationX(totalPitch * TO_RADIANS) * Matrix::CreateRotationY(totalYaw * TO_RADIANS);
				forward = finalRotation.TransformVector(Vector3::UnitZ);
				forward.Normalize();

				up = finalRotation.GetAxisY();
				right = finalRotation.GetAxisX();

				UpdateViewMatrix = true;
			}



			//Update Matrices
			CalculateViewMatrix();
			CalculateProjectionMatrix(); //Try to optimize this - should only be called once or when fov/aspectRatio changes
		}
	};
}
