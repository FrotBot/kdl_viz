#ifndef KDL_CHAIN_VIZ_HH
#define KDL_CHAIN_VIZ_HH

#include <kdl/chain.hpp>
#include <kdl/joint.hpp>

#include <GL/gl.h>

//#include <iostream>

namespace KDLCV
{
/**
		This class can be used to draw a KDL::Chain into an existing OpenGL context.
		The template parameter ChainPtrType can be used to parametrize this class 
		e.g. with your favourite shared_ptr class or simply by (KDL::Chain*).. It has
		to have an operator->() to access the pointee..

		The PoseArrayType simply has to have an operator[] to access the elements and could
		be e.g. std::vector or boost::numeric::ublas::vector or simply (double*).

		This class is only a class because of future plans to integrate geometry rendering
		which would need some state to store the geometry..
	*/
template <
	class ChainPtrType,
	class PoseArrayType>
struct KDLChainView
{
	/**
			This method draws a KDL::Chain into a OpengGL context which has
			to be setup by the user (it also has to be made current
			by the user).

			The matrix stack is used but returned to its previous state after
			the use of this method..
		*/
	void draw_chain(ChainPtrType chain, const PoseArrayType &pose)
	{
		const GLfloat g_vertex_buffer_data[] = {
			-1.0f, -1.0f, -1.0f, // triangle 1 : begin
			-1.0f, -1.0f, 1.0f,
			-1.0f, 1.0f, 1.0f, // triangle 1 : end
			1.0f, 1.0f, -1.0f, // triangle 2 : begin
			-1.0f, -1.0f, -1.0f,
			-1.0f, 1.0f, -1.0f, // triangle 2 : end
			1.0f, -1.0f, 1.0f,
			-1.0f, -1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, 1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			-1.0f, -1.0f, -1.0f,
			-1.0f, -1.0f, -1.0f,
			-1.0f, 1.0f, 1.0f,
			-1.0f, 1.0f, -1.0f,
			1.0f, -1.0f, 1.0f,
			-1.0f, -1.0f, 1.0f,
			-1.0f, -1.0f, -1.0f,
			-1.0f, 1.0f, 1.0f,
			-1.0f, -1.0f, 1.0f,
			1.0f, -1.0f, 1.0f,
			1.0f, 1.0f, 1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, 1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 1.0f,
			1.0f, 1.0f, 1.0f,
			1.0f, 1.0f, -1.0f,
			-1.0f, 1.0f, -1.0f,
			1.0f, 1.0f, 1.0f,
			-1.0f, 1.0f, -1.0f,
			-1.0f, 1.0f, 1.0f,
			1.0f, 1.0f, 1.0f,
			-1.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 1.0f};

		glPushMatrix();

		glLineWidth(3);

		// draw a coordinate system,
		glColor3f(1, 0.7, 0.7);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);
		glEnd();

		glColor3f(0.7, 1, 0.7);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
		glEnd();

		glColor3f(0.7, 0.7, 1);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
		glEnd();

		for (unsigned int segment = 0; segment < chain->getNrOfSegments(); ++segment)
		{
			//std::cout << "segment " << segment << std::endl;

			KDL::Frame pose_frame = chain->getSegment(segment).pose(pose[segment]);

			double pose_matrix[16];
			for (unsigned int i = 0; i < 16; ++i)
				pose_matrix[i] = pose_frame(i % 4, i / 4);

			// draw a line from current origin to pose which is specified by
			// the translational part of the matrix
			//glColor3f(1,1,1);
			switch (chain->getSegment(segment).getJoint().getType())
			{
			case KDL::Joint::RotAxis:
			case KDL::Joint::RotX:
			case KDL::Joint::RotY:
			case KDL::Joint::RotZ:
				glColor3f(1, 0, 0);
				break;
			case KDL::Joint::TransAxis:
			case KDL::Joint::TransX:
			case KDL::Joint::TransY:
			case KDL::Joint::TransZ:
				//draw "length" of axis here as well:
				{
					KDL::Vector axis(pose_matrix[12], pose_matrix[13], pose_matrix[14]);
					axis.Normalize();
					glColor3f(0.3, 0.3, 0.3);
					glBegin(GL_LINES);
					glVertex3f(0, 0, 0);
					glVertex3f(axis[0], axis[1], axis[2]);
					glEnd();
				}
				glColor3f(0, 0, 1);
				break;
			case KDL::Joint::None:
				glColor3f(0.5, 0.5, 0.5);
				break;
			default:
				glColor3f(1, 1, 0);
				break;
			}

			glDrawArrays(GL_TRIANGLES, 0, 12*3);
			
			glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(pose_matrix[12], pose_matrix[13], pose_matrix[14]);
			glEnd();


			// draw a line representing the rotation axis
			switch (chain->getSegment(segment).getJoint().getType())
			{
			case KDL::Joint::RotX:
				glColor3f(1, 0, 0);
				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				glVertex3f(0.1, 0, 0);
				glEnd();
				break;
			case KDL::Joint::RotY:
				glColor3f(1, 0, 0);
				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				glVertex3f(0, 0.1, 0);
				glEnd();
				break;
			case KDL::Joint::RotZ:
				glColor3f(1, 0, 0);
				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				glVertex3f(0, 0, 0.1);
				glEnd();
				break;
			case KDL::Joint::RotAxis:
			{
				glColor3f(1, 1, 0);
				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				KDL::Vector rot_axis = chain->getSegment(segment).getJoint().JointAxis() * 0.1;
				glVertex3f(rot_axis[0], rot_axis[1], rot_axis[2]);
				glEnd();
			}
			break;
			default:
				break;
			}
			glMultMatrixd(pose_matrix);
		}
		glPopMatrix();
	}

	/**
			A virtual destructor to make delete work on subclass objects behind 
			KDLChainView pointers..
		*/
	virtual ~KDLChainView() {}
};

} // namespace KDLCV

#endif
