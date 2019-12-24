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

#ifndef itkSphericalToCartesianTransform_hxx
#define itkSphericalToCartesianTransform_hxx

#include "itkSphericalToCartesianTransform.h"

namespace itk
{

template< typename TParametersValueType>
SphericalToCartesianTransform<TParametersValueType>
::SphericalToCartesianTransform():
  Superclass(ParametersDimension)
{
  this->m_Radius =  1.0 ;
}



template<typename TParametersValueType>
void
SphericalToCartesianTransform<TParametersValueType>
::PrintSelf(std::ostream &os, Indent indent) const
{
  Superclass::PrintSelf(os,indent);
  os << indent << "Radius: " << m_Radius << std::endl;
}


template<typename TParametersValueType>
typename SphericalToCartesianTransform<TParametersValueType>::OutputPointType
SphericalToCartesianTransform<TParametersValueType>
::TransformPoint(const InputPointType & inputPoint) const
{
  OutputPointType outputPoint;
  // Spherical coordinates in math convention: (r, \theta, \phi)
  // \theta = angle with x-axis, \phi = angle with z-axis and r is the radius.
  outputPoint[0] = m_Radius * std::cos(inputPoint[0]) * std::sin(inputPoint[1]); //r cos(\theta) sin(\phi)
  outputPoint[1] = m_Radius * std::sin(inputPoint[0]) * std::sin(inputPoint[1]); //r sin(\theta) sin(\phi)
  outputPoint[2] = m_Radius * std::cos(inputPoint[1]); // r cos(\phi)

  return outputPoint;
}

} // namespace itk

#endif
