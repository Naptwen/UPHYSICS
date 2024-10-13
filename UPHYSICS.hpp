#ifndef __UPHYSICS_HPP__
#define __UPHYSICS_HPP__
#include <glm/glm.hpp>

struct UCOLLIDER {
	glm::vec3 originalCenter = glm::vec3(0.0f);
	glm::quat originRotate = glm::quat();
	glm::vec3 originScale = glm::vec3(1.0f);
	glm::vec4 originColor = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
	bool isCollision = false;
	virtual ~UCOLLIDER() = default;
};

struct USHAPE3D : public UCOLLIDER
{
	std::vector<glm::vec3> originVertiecs;
	std::vector<glm::vec3> convexVertices;
};
  
struct UBOX3D : public UCOLLIDER
{
	glm::vec3 center = glm::vec3(0.0f);
	glm::vec3 axes[3] = {
		glm::vec3(1, 0, 0),
		glm::vec3(0, 1, 0),
		glm::vec3(0, 0, 1)
	};
	glm::vec3 halfLengths = glm::vec3(1.0f);
};

struct USPHERE3D : public UCOLLIDER
{
	glm::vec3 center = glm::vec3(0.0f);
	float radius = 1.0f;
};

struct UCAPSULE3D : public UCOLLIDER
{
	glm::vec3 center = glm::vec3(0.0f);
	glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	float radius = 1.0f;
	float height = 1.0f;
};

struct TRANSFORMATION
{
	glm::vec3 position = glm::vec3(0.0f);
	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 scale = glm::vec3(1.0f);
};

struct NEWTON
{
	float mass = 1.0f;
	glm::vec3 force = glm::vec3(0.0f);
	glm::vec3 acceleration  = glm::vec3(0.0f);
	glm::vec3 velocity = glm::vec3(0.0f);
};

namespace UCONTROLLER {
	const glm::vec3 zfront = glm::vec3(0.0f, 0.0f, 1.0f);

	inline void MOVE_FORWARD(std::shared_ptr<TRANSFORMATION> trans, float velocity) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, zfront));
		trans->position += front * velocity;
	}
	inline void MOVE_BACKWARD(std::shared_ptr<TRANSFORMATION> trans, float velocity) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, zfront));
		trans->position -= front * velocity;
	}
	inline void MOVE_LEFT(std::shared_ptr<TRANSFORMATION> trans, float velocity) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, zfront));
		glm::vec3 right = glm::normalize(cross(front, trans->up));
		trans->position -= right * velocity;
	}
	inline void MOVE_RIGHT(std::shared_ptr<TRANSFORMATION> trans, float velocity) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, zfront));
		glm::vec3 right = glm::normalize(cross(front, trans->up));
		trans->position += right * velocity;
	}
	inline void ROTATE(std::shared_ptr<TRANSFORMATION> trans, const float yawDelta, const float pitchDelta) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, zfront));
		glm::quat yawQuat = glm::angleAxis(glm::radians(yawDelta), glm::vec3({ 0.0f, 1.0f, 0.0f }));
		glm::vec3 right = glm::normalize(cross(front, trans->up));
		glm::quat pitchQuat = glm::angleAxis(glm::radians(pitchDelta), right);
		trans->rotation = yawQuat * pitchQuat * trans->rotation;
	}
}
namespace UPHYSICS {
	namespace OBB {
		inline void project(const std::shared_ptr<UBOX3D> ubox, const glm::vec3& axis, float& min, float& max)
		{
			assert(ubox->halfLengths.x >= 0 && ubox->halfLengths.y >= 0 && ubox->halfLengths.z >= 0);
			float d = glm::dot(ubox->center, axis);
			//axes has the rotation of the box
			//using this info we can project the box to the axis
			glm::vec3 p = glm::abs(glm::vec3(
				glm::dot(axis, ubox->axes[0]),
				glm::dot(axis, ubox->axes[1]),
				glm::dot(axis, ubox->axes[2])
			));
			//noe the axes are scaled by the half lengths
			float r = glm::dot(ubox->halfLengths, p);
			//the min and max are the projection of the center of the box
			min = d - r;
			max = d + r;
		}

		template<typename T1>
		inline void updateCollisionTransform(std::shared_ptr<T1> collider, const std::shared_ptr<TRANSFORMATION> trans) {}
		inline void updateCollisionTransform(std::shared_ptr<UBOX3D> collider, const std::shared_ptr<TRANSFORMATION> trans)
		{
			collider->center = trans->position + collider->originalCenter;
			collider->axes[0] = trans->rotation * collider->originRotate * glm::vec3(1, 0, 0);
			collider->axes[1] = trans->rotation * collider->originRotate * glm::vec3(0, 1, 0);
			collider->axes[2] = trans->rotation * collider->originRotate * glm::vec3(0, 0, 1);
			collider->halfLengths = collider->originScale * trans->scale;
		}
		inline void updateCollisionTransform(std::shared_ptr<USPHERE3D> collider, const std::shared_ptr<TRANSFORMATION> trans)
		{
			collider->center = glm::vec3(trans->position);
		}
		inline void updateCollisionTransform(std::shared_ptr<UCAPSULE3D> collider, const std::shared_ptr<TRANSFORMATION> trans)
		{
			collider->center = trans->position + collider->originalCenter;
			collider->rotation = trans->rotation * collider->originRotate;
			collider->radius = glm::length(trans->scale) * glm::length(collider->originScale.x + collider->originScale.z);
			collider->height = glm::length(trans->scale) * glm::length(collider->originScale.y);
		}

		template<typename T1, typename T2>
		inline bool IsCollisionRay(const glm::vec3& rayOrigin, const glm::vec3& rayEnd, const std::shared_ptr<T1> collider, glm::vec3& hitPoint) { return false; }
		inline bool IsCollisionRay(const glm::vec3& rayOrigin, const glm::vec3& rayEnd, const std::shared_ptr<UBOX3D> collider, glm::vec3& hitPoint)
		{
			glm::vec3 rayDir = glm::normalize(rayEnd - rayOrigin);

			float rayStartProj[3] = {
				dot(rayOrigin - collider->center, collider->axes[0]),
				dot(rayOrigin - collider->center, collider->axes[1]),
				dot(rayOrigin - collider->center, collider->axes[2])
			};
			float rayEndProj[3] = {
				dot(rayEnd - collider->center, collider->axes[0]),
				dot(rayEnd - collider->center, collider->axes[1]),
				dot(rayEnd - collider->center, collider->axes[2])
			};
			float tMin = std::numeric_limits<float>::max();
			for (int i = 0; i < 3; ++i) {
				float boxMin = -collider->halfLengths[i];
				float boxMax = collider->halfLengths[i];
				float rayptrMin = std::min(rayStartProj[i], rayEndProj[i]);
				float rayptrMax = std::max(rayStartProj[i], rayEndProj[i]);
				bool intersection_minmax = (boxMin <= rayptrMax && boxMax >= rayptrMin);
				if (!intersection_minmax) return false;
				float t1 = abs(boxMin - rayStartProj[i]);
				float t2 = abs(boxMax - rayStartProj[i]);
				tMin = std::min({ tMin, t1, t2 });
			}
			hitPoint = rayOrigin + tMin * rayDir;
			return true;
		}
		inline bool IsCollisionRay(const glm::vec3& rayOrigin, const glm::vec3& rayEnd, const std::shared_ptr<USPHERE3D> collider, glm::vec3& hitPoint) {
			glm::vec3 rayDir = rayEnd - rayOrigin;
			float rayLength = length(rayDir);
			rayDir = glm::normalize(rayDir);

			glm::vec3 L = collider->center - rayOrigin;
			float tca = glm::dot(L, rayDir);
			if (tca < 0) return false;

			float d2 = glm::length2(L) - glm::length2(tca);
			float radiusSquared = collider->radius * collider->radius;
			if (d2 > radiusSquared) return false;

			float thc = std::sqrt(radiusSquared - d2);
			float t0 = tca - thc;
			float t1 = tca + thc;

			if (t0 > rayLength || t1 < 0) return false;

			return true;
		}
		inline bool IsCollisionRay(const glm::vec3& rayOrigin, const glm::vec3& rayEnd, const std::shared_ptr<UCAPSULE3D> collider, glm::vec3& hitPoint) {
			return false;
		}

		template<typename T1, typename T2>
		inline bool isCollision(const std::shared_ptr<T1> A, const std::shared_ptr<T2> B, glm::vec3& normal) { return false; }
		inline bool isCollision(const std::shared_ptr<UBOX3D> A, const std::shared_ptr<UBOX3D> B, glm::vec3& penetration)
		{
			glm::vec3 total_axes[15] = {
				A->axes[0],
				A->axes[1],
				A->axes[2],
				B->axes[0],
				B->axes[1],
				B->axes[2],
				glm::cross(A->axes[0], B->axes[0]),
				glm::cross(A->axes[0], B->axes[1]),
				glm::cross(A->axes[0], B->axes[2]),
				glm::cross(A->axes[1], B->axes[0]),
				glm::cross(A->axes[1], B->axes[1]),
				glm::cross(A->axes[1], B->axes[2]),
				glm::cross(A->axes[2], B->axes[0]),
				glm::cross(A->axes[2], B->axes[1]),
				glm::cross(A->axes[2], B->axes[2])
			};

			float minPenetration = std::numeric_limits<float>::max();
			glm::vec3 penetrationAxis = glm::vec3(0.0f);

			for (auto& axis : total_axes) {
				if (glm::length2(axis) < 1e-10f) continue;
				axis = glm::normalize(axis);

				float minA, maxA, minB, maxB;
				UPHYSICS::OBB::project(A, axis, minA, maxA);
				UPHYSICS::OBB::project(B, axis, minB, maxB);

				if (maxA < minB || maxB < minA) return false;

				float overlap = std::min(maxA, maxB) - std::max(minA, minB);
				if (overlap < minPenetration) {
					minPenetration = overlap;
					penetrationAxis = axis;
				}
			}

			glm::vec3 localAcenter = B->center - A->center;
			if (glm::dot(localAcenter, penetrationAxis) > 0)
				penetrationAxis = -penetrationAxis;

			penetration = minPenetration * penetrationAxis;
			return true;
		}
		inline bool isCollision(const std::shared_ptr<UBOX3D> A, const std::shared_ptr<USPHERE3D> B, glm::vec3& penetration)
		{
			glm::vec3 sphereCenter = B->center - A->center;

			glm::vec3 localSphereCenter = {
				dot(sphereCenter, A->axes[0]),
				dot(sphereCenter, A->axes[1]),
				dot(sphereCenter, A->axes[2])
			};

			glm::vec3 closestPoint = {
				std::min(std::max(localSphereCenter[0], -A->halfLengths[0]), A->halfLengths[0]),
				std::min(std::max(localSphereCenter[1], -A->halfLengths[1]), A->halfLengths[1]),
				std::min(std::max(localSphereCenter[2], -A->halfLengths[2]), A->halfLengths[2])
			};

			glm::vec3 localPenetrationDir = localSphereCenter - closestPoint;

			float distanceSquared = length(localPenetrationDir);
			float distance = std::sqrt(distanceSquared);

			if (distanceSquared <= (B->radius * B->radius)) {
				if (distance > 0) {
					glm::vec3 worldPenetrationDir = glm::vec3(0);
					for (int i = 0; i < 3; i++) {
						worldPenetrationDir += A->axes[i] * localPenetrationDir[i];
					}
				}
				else {
				}
				return true;
			}
			return false;
		}
		inline bool isCollision(const std::shared_ptr<UBOX3D> A, const std::shared_ptr<UCAPSULE3D> B, glm::vec3& penetration)
		{
			// Transform capsule to box space
			glm::vec3 capsuleAxis = B->rotation * glm::vec3(0, 1, 0);
			glm::vec3 capsuleTopPoint = B->center + capsuleAxis * B->height * 0.5f;
			glm::vec3 capsuleBottomPoint = B->center - capsuleAxis * B->height * 0.5f;
			glm::vec3 relCapsuleCenter = B->center - A->center;
			glm::vec3 relCapsuleTop = capsuleTopPoint - A->center;
			glm::vec3 relCapsuleBottom = capsuleBottomPoint - A->center;

			float localCapsuleCenter[3] = {
				dot(relCapsuleCenter, A->axes[0]),
				dot(relCapsuleCenter, A->axes[1]),
				dot(relCapsuleCenter, A->axes[2])
			};

			float localCapsuleTop[3] = {
				dot(relCapsuleTop, A->axes[0]),
				dot(relCapsuleTop, A->axes[1]),
				dot(relCapsuleTop, A->axes[2])
			};

			float localCapsuleBottom[3] = {
				dot(relCapsuleBottom, A->axes[0]),
				dot(relCapsuleBottom, A->axes[1]),
				dot(relCapsuleBottom, A->axes[2])
			};

			float minPenetration = std::numeric_limits<float>::max();
			int minPenetrationAxis = -1;

			for (int i = 0; i < 3; ++i) {
				float boxMin = -A->halfLengths[i];
				float boxMax = A->halfLengths[i];
				float capsuleMin = std::min(localCapsuleTop[i], localCapsuleBottom[i]) - B->radius;
				float capsuleMax = std::max(localCapsuleTop[i], localCapsuleBottom[i]) + B->radius;
				bool intersection_minmax = (boxMin <= capsuleMax && boxMax >= capsuleMin);
				if (!intersection_minmax) return false;

				float overlap = std::min({ abs(capsuleMin - boxMin),abs(capsuleMin - boxMax),
										   abs(capsuleMax - boxMin),abs(capsuleMax - boxMax) });
				if (overlap < minPenetration) {
					minPenetration = overlap;
					minPenetrationAxis = i;
				}
			}
			if (minPenetrationAxis == -1) return false;
			penetration = A->axes[minPenetrationAxis] * minPenetration;
			if (glm::dot(penetration, relCapsuleCenter) > 0)
				penetration = -penetration;
			return true;
		}
		inline bool isCollision(const std::shared_ptr<USHAPE3D> A, const std::shared_ptr<USHAPE3D> B, glm::vec3& penetration) {
			//GJK
			return false;
		}
	}
	namespace GJK {
		inline void updateCollisionTransform(std::shared_ptr<USHAPE3D> shape, const std::shared_ptr<TRANSFORMATION> trans) {
			for (size_t i = 0; i < shape->convexVertices.size(); ++i) {
				shape->convexVertices[i] = trans->scale * shape->originScale * shape->originVertiecs[i];
				shape->convexVertices[i] = trans->rotation * shape->originRotate * shape->convexVertices[i];
				shape->convexVertices[i] += trans->position + shape->originalCenter;
			}
		}
		inline glm::vec3 furthestPoint(const glm::vec3& d, const std::vector<glm::vec3>& X) {
			assert(X.size() > 0);
			glm::vec3 maxPoint = X.at(0);
			float max = glm::dot(X.at(0), d);
			for (int i = 1; i < X.size(); ++i) {
				float dot = glm::dot(X.at(i), d);
				if (dot > max) {
					max = dot;
					maxPoint = X.at(i);
				}
			}
			return maxPoint;
		}
		template<typename T>
		inline int argmin(const std::vector<T> list) {
			T min = std::min_element(list.begin(), list.end());
			return std::distance(list.begin(), min);
		}
		inline glm::vec3 support(const glm::vec3 d, const std::vector<glm::vec3> A, const std::vector<glm::vec3> B) {
			return furthestPoint(d, A) - furthestPoint(-d, B);
		}
		inline glm::vec3 faceNorm(const glm::vec3 A, const glm::vec3 B, const glm::vec3 C) {
			glm::vec3 BA = B - A;
			glm::vec3 CA = C - A;
			while (glm::length(glm::cross(BA, CA)) < 1e-6) {
				std::random_device rd;
				std::mt19937 gen(rd());
				std::uniform_real_distribution<float> dis(-1e-4, 1e-4);

				BA = B + glm::vec3(dis(gen), dis(gen), dis(gen)) - A;
				CA = C + glm::vec3(dis(gen), dis(gen), dis(gen)) - A;
			}
			return glm::normalize(glm::cross(BA, CA));
		}
		inline bool handleSimplex3D(std::vector<glm::vec3>& simplex, glm::vec3& dir) {
			assert(simplex.size() >= 2);
			if (simplex.size() == 2) {
				glm::vec3& B = simplex[1];
				glm::vec3& A = simplex[0];
				glm::vec3 BA = A - B;
				glm::vec3 BO = -B;
				dir = glm::cross(glm::cross(BA, BO), BA);
			}
			else if (simplex.size() == 3) {
				glm::vec3& A = simplex[0];
				glm::vec3& B = simplex[1];
				glm::vec3& C = simplex[2]; //C is away the origin by perp AB
				glm::vec3 CB = B - C;
				glm::vec3 CA = A - C;
				dir = glm::cross(CB, CA);
				if (glm::dot(dir, -C) < 0) 
					dir = -dir;
			}
			else if (simplex.size() == 4) {
				glm::vec3& A = simplex[0];
				glm::vec3& B = simplex[1];
				glm::vec3& C = simplex[2];
				glm::vec3& D = simplex[3];
				glm::vec3 DA = A - D;
				glm::vec3 DB = B - D;
				glm::vec3 DC = C - D;
				glm::vec3 DO = -D;
				glm::vec3 DBA = normalize(glm::cross(DB, DA));
				glm::vec3 DCB = normalize(glm::cross(DC, DB));
				glm::vec3 DAC = normalize(glm::cross(DA, DC));
				//Since ABC is cyclic 
				bool isDBA = glm::dot(DBA, DO) > 0;
				bool isDCB = glm::dot(DCB, DO) > 0;
				bool isDAC = glm::dot(DAC, DO) > 0;
				//if the D-> cyclic has the same dot to DO then it has the origin
				if (isDBA != isDCB && isDBA != isDAC) {
					simplex.erase(simplex.begin() + 2);
					dir = isDBA > 0 ? DBA : -DBA;
					return false;
				}
				else if (isDCB != isDAC && isDCB != isDBA) {
					simplex.erase(simplex.begin());
					dir = isDCB > 0 ? DCB : -DCB;
					return false;
				}
				else if (isDAC != isDBA && isDAC != isDCB) {
					simplex.erase(simplex.begin() + 1);
					dir = isDAC > 0 ? DAC : -DAC;
					return false;
				}
				return true;
			}
			return false;
		}
		inline bool GJK(const std::vector<glm::vec3> A, 
						const std::vector<glm::vec3> B,
						std::vector<glm::vec3>& mikovskydiff,
						std::vector<glm::vec3>& simplex) {

			if (mikovskydiff.size() != A.size() * B.size())
				mikovskydiff.resize(A.size() * B.size());

			int k = 0;
			for (const auto& a : A) {
				for (const auto& b : B) {
					mikovskydiff[k++] = a - b;
				}
			}

			glm::vec3 d = { 1, 0, 0 };
			simplex.clear();
			glm::vec3 p = support(d, A, B);
			simplex.push_back(p);
			d = -p;
			int iteration = 0;
			while (iteration++ < 100) {
				d = glm::normalize(d);
				p = support(d, A, B);
				if (glm::dot(p, d) < 0) {
					return false;
				}
				simplex.push_back(p);
				if (handleSimplex3D(simplex, d))
					break;
			} 
			if (iteration >= 100)
				return false;
			assert(simplex.size() == 4);
			return true;
		}
		inline glm::vec3 EPA3D(const std::vector<glm::vec3>& minkowskiDiff, const std::vector<glm::vec3>& simplex) {
			//only for GJK is true then the origin inside the simplex
			//Another condition for it is that that only when the full simplex is formed
			//Since the perfection of convergence is guaranteed in Minkowski dimension
			assert(simplex.size() == 4);
			glm::vec3 A = simplex[0];
			glm::vec3 B = simplex[1];
			glm::vec3 C = simplex[2];
			glm::vec3 D = simplex[3];
			glm::vec3 normABC = faceNorm(A, B, C);
			glm::vec3 normBCD = faceNorm(B, C, D);
			glm::vec3 normCDA = faceNorm(C, D, A);
			glm::vec3 normDAB = faceNorm(D, A, B);			
			//To find the opposite direction of the normal vector to find extended face
			normABC = glm::dot(-A, normABC) < 0 ? normABC : -normABC;
			normBCD = glm::dot(-B, normBCD) < 0 ? normBCD : -normBCD;
			normCDA = glm::dot(-C, normCDA) < 0 ? normCDA : -normCDA;
			normDAB = glm::dot(-D, normDAB) < 0 ? normDAB : -normDAB;
			//Since we need only the distnace from origin to the normal vector
			float dABC = glm::dot(A, normABC);
			float dBCD = glm::dot(B, normBCD);
			float dCDA = glm::dot(C, normCDA);
			float dDBA = glm::dot(D, normDAB);
			// Find the minimum distanc
			std::vector<glm::vec3> simpleList = { A, B, C, D };
			std::map<float, std::array<glm::vec3, 4>> infoMap = {
				{ dABC, {normABC, A, B, C}},
				{ dBCD, {normBCD, B, C, D}},
				{ dCDA, {normCDA, C, D, A}},
				{ dDBA, {normDAB, D, A, B}}
			};
			int iteration = 0;
			while (iteration++ < 100) {
				float maxDis = infoMap.begin()->first;
				glm::vec3 minNorm = infoMap.begin()->second[0];
				glm::vec3 farPoint = infoMap.begin()->second[1];
				if (isnan(minNorm.x))
					std::cout << "NAN" << std::endl;
				if (isnan(farPoint.x))
					std::cout << "NAN" << std::endl;
				for (auto& point : minkowskiDiff) {
					float dis = glm::dot(point, minNorm);
					if (dis > maxDis) {
						maxDis = dis;
						farPoint = point;
					}
				}
				//Find the new face
				for(auto& simple : simpleList)
					if (simple == farPoint)
						goto finish;
				simpleList.push_back(farPoint);
				//Find the new normal vector
				glm::vec3 X = infoMap.begin()->second[1];
				glm::vec3 Y = infoMap.begin()->second[2];
				glm::vec3 Z = infoMap.begin()->second[3];
				glm::vec3 newNormXY = faceNorm(farPoint, X, Y);
				glm::vec3 newNormYZ = faceNorm(farPoint, Y, Z);
				glm::vec3 newNormZX = faceNorm(farPoint, Z, X);
				newNormXY = glm::dot(-farPoint, newNormXY) < 0 ? newNormXY : -newNormXY;
				newNormYZ = glm::dot(-farPoint, newNormYZ) < 0 ? newNormYZ : -newNormYZ;
				newNormZX = glm::dot(-farPoint, newNormZX) < 0 ? newNormZX : -newNormZX;
				float disXY = glm::dot(farPoint, newNormXY);
				float disYZ = glm::dot(farPoint, newNormYZ);
				float disZX = glm::dot(farPoint, newNormZX);

				infoMap.erase(infoMap.begin());
				infoMap[disXY] = { newNormXY, farPoint, X, Y };
				infoMap[disYZ] = { newNormYZ, farPoint, Y, Z };
				infoMap[disZX] = { newNormZX, farPoint, Z, X };
			}
			if (iteration >= 100)
				return glm::vec3(0.0f);
		finish:
			if (isnan((infoMap.begin()->second[0] * infoMap.begin()->first).x))
				std::cout << "NAN" << std::endl;
			return infoMap.begin()->second[0] * infoMap.begin()->first;
		}
		inline bool isCollision(std::shared_ptr<USHAPE3D> A, std::shared_ptr<USHAPE3D> B, 
			std::vector<glm::vec3>& mikovskydiff, std::vector<glm::vec3>& simplex,
			glm::vec3& penetration) {
			if (UPHYSICS::GJK::GJK(A->convexVertices, B->convexVertices, mikovskydiff, simplex)) {
				penetration = UPHYSICS::GJK::EPA3D(mikovskydiff, simplex);
				return true;
			}
			return false;
		}
	}
	inline glm::vec3 GET_FRONT(std::shared_ptr<TRANSFORMATION> trans) {
		return glm::normalize(rotate(trans->rotation, UCONTROLLER::zfront));
	}
	inline glm::vec3 GET_RIGHT(std::shared_ptr<TRANSFORMATION> trans) {
		glm::vec3 front = glm::normalize(rotate(trans->rotation, UCONTROLLER::zfront));
		return glm::normalize(cross(front, trans->up));
	}
	inline void FORCE(std::shared_ptr<NEWTON> newton, glm::vec3 normal_vec = glm::vec3(0.0f)) {
		if (newton->force == glm::vec3(0.0f)) return;
		if (glm::length2(newton->velocity) > 0.0f) {
			float friction_coef = 0.051f;
			float normal_force = newton->mass;
			glm::vec3 friction_dir = (glm::length2(newton->velocity) > 0.0f) ? glm::normalize(newton->velocity) : glm::vec3(0.0f);
			glm::vec3 friction_force = friction_dir * friction_coef * normal_force;
			newton->force.x = (abs(newton->force.x) < abs(friction_force.x)) ? 0.0f : newton->force.x - friction_force.x;
			newton->force.y = (abs(newton->force.y) < abs(friction_force.y)) ? 0.0f : newton->force.y - friction_force.y;
			newton->force.z = (abs(newton->force.z) < abs(friction_force.z)) ? 0.0f : newton->force.z - friction_force.z;
		}
		newton->acceleration = newton->force / newton->mass;
	}
	inline void MOMENTUM(std::shared_ptr<NEWTON> newton, float dampping = 0.99f) {
		newton->velocity = newton->acceleration;
	}
	inline void MOVE(std::shared_ptr<TRANSFORMATION> trans, std::shared_ptr<NEWTON> newton) {
		trans->position += newton->velocity;
	}

	inline void updateCollisionResolutionSimple(std::shared_ptr<TRANSFORMATION> trans, std::shared_ptr<NEWTON> newton, glm::vec3& penetration) {
		trans->position.x += (newton->velocity.x > 0) ? -penetration[0] : penetration[0];
		trans->position.y += (newton->velocity.y > 0) ? -penetration[1] : penetration[1];
		trans->position.z += (newton->velocity.z > 0) ? -penetration[2] : penetration[2];
	}
}


#endif