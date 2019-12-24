//**********************************************************
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//**********************************************************
//
//
#ifndef itkCartesianToSphericalTransform_h
#define itkCartesianToSphericalTransform_h

#include <iostream>
#include "itkTransform.h"
#include "itkExceptionObject.h"
#include "itkMatrix.h"

namespace itk
{

/** \class SphericalToCartesianTransform
 *
 * \brief Spherical to Cartesian Coordinate Transformation(e.g. space coordinates).
 *
 * \f[          x_1 = r sin( \phi ) cos( \theta ) \f]
 * \f[          x_2 = r sin( \phi ) sin( \theta ) \f]
 * \f[          x_3 = r cos( \phi ) \f]
 *
 * \par
 * Radius of the Angle transform  can be specified with SetRadius().
 * The default is center of coordinate system < 0, 0, 0 >.
 *
 * Dimension must be 3.
 *
 * \phi change from 0 to \pi while \theta change from 0 to 2\pi
 *
 * \author HaaPut,
 *
 * \ingroup Transforms
 * \ingroup CoordinateTransform
 */
template < typename TParametersValueType=double >        // Number of dimensions
class ITK_TEMPLATE_EXPORT CartesianToSphericalTransform:
  public Transform< TParametersValueType, 3, 2 >
{
public:
  ITK_DISALLOW_COPY_AND_ASSIGN(CartesianToSphericalTransform);

  /** Standard class type alias. */
  using Self = CartesianToSphericalTransform;
  using Superclass = Transform< TParametersValueType, 3, 2 >;
  using Pointer = SmartPointer< Self >;
  using ConstPointer = SmartPointer< const Self >;

  /** New macro for creation of through the object factory.*/
  itkNewMacro( Self );

  /** Run-time type information (and related methods). */
  itkTypeMacro( CartesianToSphericalTransform, Transform );


  /** Dimension of the domain space. */
  static constexpr unsigned int ParametersDimension = 0;

  /** Standard scalar type for this class. */
  using ScalarType = typename Superclass::ScalarType;

  /** Standard Jacobian container. */
  using JacobianType = typename Superclass::JacobianType;
//
  /** Standard parameters container. */
  using ParametersType = typename Superclass::ParametersType;

  /** Fixed Parameter type */
  using FixedParametersType = typename Superclass::FixedParametersType;

  /** Standard vector type for this class. */
  using InputVectorType = Vector<TParametersValueType, itkGetStaticConstMacro(InputSpaceDimension)>;
  using OutputVectorType = Vector<TParametersValueType, itkGetStaticConstMacro(OutputSpaceDimension)>;

  /** Standard covariant vector type for this class. */
  using InputCovariantVectorType = CovariantVector<TParametersValueType, itkGetStaticConstMacro(InputSpaceDimension)>;
  using OutputCovariantVectorType = CovariantVector<TParametersValueType, itkGetStaticConstMacro(OutputSpaceDimension)>;

  /** Standard vnl_vector type for this class. */
  using InputVnlVectorType = vnl_vector_fixed<TParametersValueType, itkGetStaticConstMacro(InputSpaceDimension)>;
  using OutputVnlVectorType = vnl_vector_fixed<TParametersValueType, itkGetStaticConstMacro(OutputSpaceDimension)>;

  /** Standard coordinate point type for this class. */
  using InputPointType = Point<TParametersValueType, itkGetStaticConstMacro(InputSpaceDimension)>;
  using OutputPointType = Point<TParametersValueType, itkGetStaticConstMacro(OutputSpaceDimension)>;

  /** Method to transform a point.
   * This method transforms a point from spherical
   * coordinates <\rho, \theta, \phi> to cartesian coordinates.
   */
  OutputPointType TransformPoint(const InputPointType  &point ) const override;

  /** Method to transform a vector - not applicable for this type of transform. */
  OutputVectorType TransformVector(const InputVectorType &) const override
    {
    itkExceptionMacro(<< "Method not applicable for this type of transform.");
    return OutputVectorType();
    }

  /** Method to transform a vnl_vector - not applicable for this type of transform. */
  OutputVnlVectorType TransformVector(const InputVnlVectorType &) const override
    {
    itkExceptionMacro(<< "Method not applicable for this type of transform.");
    return OutputVnlVectorType();
    }

  /** Method to transform a vector - not applicable for this type of transform. */
  typename Superclass::OutputVectorPixelType TransformVector(
      const typename Superclass::InputVectorPixelType &, const InputPointType &) const override
    {
    itkExceptionMacro(<< "Method not applicable for this type of transform.");
    return typename Superclass::OutputVectorPixelType();
    }

  using Superclass::TransformVector;

  /** Method to transform a CovariantVector - not applicable for this type of transform */
  OutputCovariantVectorType TransformCovariantVector(
    const InputCovariantVectorType &) const override
    {
    itkExceptionMacro(<< "Method not applicable for this type of transform.");
    return OutputCovariantVectorType();
    }

  using Superclass::TransformCovariantVector;

  void ComputeJacobianWithRespectToParameters( const InputPointType &, JacobianType & ) const override
    {
    itkExceptionMacro(<< "Method not implemented yet.");
    }

  void SetParameters(const ParametersType &) override {}

  void SetFixedParameters(const FixedParametersType &) override {}

  itkGetMacro(Radius, double);
  itkSetMacro(Radius, double);

  itkGetMacro(Eps, double);
  itkSetMacro(Eps, double);

protected:
  CartesianToSphericalTransform();
  ~CartesianToSphericalTransform() = default;

  /** Print contents of an SphericalToCartesianTransform. */
  void PrintSelf(std::ostream &os, Indent indent) const override;

private:
    const double m_PI =  std::atan(1.0)*4;
    double m_Radius;
    double m_Eps;

    }; //class CartesianToSphericalTransform

}  // namespace itk


#ifndef ITK_MANUAL_INSTANTIATION
#include "itkCartesianToSphericalTransform.hxx"
#endif

#endif
