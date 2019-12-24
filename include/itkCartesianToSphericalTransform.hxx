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

#ifndef itkCartesianToSphericalTransform_hxx
#define itkCartesianToSphericalTransform_hxx

#include "itkCartesianToSphericalTransform.h"

namespace itk
{

    template<typename TParametersValueType>
    CartesianToSphericalTransform<TParametersValueType>
    ::CartesianToSphericalTransform():
    Superclass(ParametersDimension)
    {
        this->m_Radius = 1.0;
        this->m_Eps = 0.1;
    }


template<typename TParametersValueType>
void
CartesianToSphericalTransform<TParametersValueType>
::PrintSelf(std::ostream &os, Indent indent) const
{
  Superclass::PrintSelf(os,indent);
}


template<typename TParametersValueType>
typename CartesianToSphericalTransform<TParametersValueType>::OutputPointType
CartesianToSphericalTransform<TParametersValueType>
::TransformPoint(const InputPointType & inputPoint) const
{
  OutputPointType outputPoint;
  outputPoint.Fill(std::numeric_limits<typename OutputPointType::ValueType>::max());
  double r = 0;
  for(unsigned d = 0; d < 3 ; ++d){
      r += inputPoint[d]*inputPoint[d];
  }
  r = std::sqrt(r);
  // map  points along sphere of radius m_Radius to corresponding point of sphere.
  // else map to infinity..
  if (std::abs(r - m_Radius) <= m_Eps) {
      outputPoint[1] = std::acos(inputPoint[2] / r); //phi = acos(z / r)
      double theta = std::atan2(inputPoint[1], inputPoint[0]);
      theta = theta < 0 ? 2 * m_PI + theta : theta;
      outputPoint[0] = theta;
  }
  return outputPoint;
}

} // namespace itk

#endif
