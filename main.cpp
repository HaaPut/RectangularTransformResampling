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
#include <iostream>
#include <itkImage.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkImageFileWriter.h>
#include "itkSphericalToCartesianTransform.h"
#include "itkMyResampleImageFilter.h"
#include "itkCartesianToSphericalTransform.h"

void forwardPointTest(){
    using SphericalToCartTransformType = itk::SphericalToCartesianTransform<double>;
    SphericalToCartTransformType::Pointer c2SphericalTx = SphericalToCartTransformType::New();
    c2SphericalTx->SetRadius(2);
    SphericalToCartTransformType::InputPointType point;
    double theta = 45, phi = 45;

    point[0] = theta*(3.14/180);
    point[1] = phi*(3.14/180);
    c2SphericalTx->TransformPoint(point);
    std::cout << point << " -> " << c2SphericalTx->TransformPoint(point) << "\n";
}

int forwardImageMappingTest(){

    using InputImageType = itk::Image<float, 2>;
    InputImageType::Pointer spherical = InputImageType::New();
    double thetaStep = 0.5, phiStep = 0.5;
    InputImageType::SizeType size;
    size[0] = 360/thetaStep;
    size[1] = 180/phiStep;

    InputImageType::PointType origin;
    origin.Fill(0.0);
    InputImageType::SpacingType spacing;
    spacing[0] = (3.14159/180)*thetaStep;
    spacing[1] = (3.14159/180)*phiStep;

    spherical->SetOrigin(origin);
    spherical->SetSpacing(spacing);
    spherical->SetRegions(size);//sphericalRegion);
    spherical->Allocate();

    itk::ImageRegionIteratorWithIndex<InputImageType> iter(spherical, spherical->GetLargestPossibleRegion());
    InputImageType::PointType physicalPoint;
    iter.GoToBegin();
    while (!iter.IsAtEnd()){
        spherical->TransformIndexToPhysicalPoint(iter.GetIndex(), physicalPoint);
        double value = std::sin(3*physicalPoint[0]) + 1; //sin(theta)
        iter.Set(value);
        ++iter;
    }
    using SphericalWriterType = itk::ImageFileWriter<InputImageType>;
    SphericalWriterType::Pointer sphericalWriter = SphericalWriterType::New();
    sphericalWriter->SetInput(spherical);
    sphericalWriter->SetFileName("spherical.tif");
    sphericalWriter->Update();

    using CartToSphericalTransformType = itk::CartesianToSphericalTransform<double>;
    CartToSphericalTransformType::Pointer sphericalToCartTx = CartToSphericalTransformType::New();
    sphericalToCartTx->SetRadius(2.0);

    using OutputImageType = itk::Image<float, 3 >;
    using ResampleFilterType = itk::MyResampleImageFilter<InputImageType, OutputImageType>;
    ResampleFilterType::Pointer resampler = ResampleFilterType::New();

    ResampleFilterType::SizeType outputSize;
    outputSize.Fill(100);

    ResampleFilterType::OutputPointType outputOrigin;
    outputOrigin.Fill(-5.0);

    OutputImageType::IndexType outputStartIndex;
    outputStartIndex.Fill(0);

    OutputImageType::SpacingType outputSpacing;
    outputSpacing.Fill(0.1);

    resampler->SetDefaultPixelValue(-0.1);
    resampler->SetInput(spherical);
    resampler->SetTransform(sphericalToCartTx);
    resampler->SetOutputSpacing(outputSpacing);
    resampler->SetOutputOrigin(outputOrigin);
    resampler->SetOutputStartIndex(outputStartIndex);
    resampler->SetSize(outputSize);

    using CartesianWriterType = itk::ImageFileWriter<OutputImageType >;
    CartesianWriterType::Pointer cartesianWriter = CartesianWriterType::New();

    cartesianWriter->SetInput(resampler->GetOutput());
    cartesianWriter->SetFileName("cartesian.tif");
    cartesianWriter->Update();
    return 0;

}

int main() {
    forwardPointTest();
    forwardImageMappingTest();
    return 0;
}
