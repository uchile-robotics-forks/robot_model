// Copyright (c) 2010, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Tim Field */

#include "collada_urdf/collada_urdf.h"

#include <gtest/gtest.h>

#include <boost/filesystem/path.hpp>

#include <fstream>
#include <sstream>
#include <string>

class ColladaUrdfTestFixture : public ::testing::Test {
public:
  std::string test_urdf_string = "test/pr2.urdf";
  std::string test_collada_string = "test/pr2.dae";

  void SetUp() {
      boost::filesystem::path urdfPath = boost::filesystem::current_path();
      urdfPath += "/test/pr2.urdf";

      if (!boost::filesystem::exists(urdfPath)) {
         std::cerr << "URDF file did not exist." << std::endl;
         FAIL();
      }
      test_urdf_string = urdfPath.string();
  }

  std::string readTestUrdfString() {
      std::ifstream file(test_urdf_string);
      std::stringstream ss;
      ss << file.rdbuf();
      return ss.str();
  }
};


TEST_F(ColladaUrdfTestFixture, collada_from_urdf_file_works)
{
    boost::shared_ptr<DAE> dom;
    ASSERT_TRUE(collada_urdf::colladaFromUrdfFile(test_urdf_string, dom));
    ASSERT_TRUE(collada_urdf::colladaToFile(dom, test_collada_string));
}

TEST_F(ColladaUrdfTestFixture, collada_from_urdf_string_works)
{
    std::string urdf_str = readTestUrdfString();

    boost::shared_ptr<DAE> dom;
    ASSERT_TRUE(collada_urdf::colladaFromUrdfString(urdf_str, dom));
    ASSERT_TRUE(collada_urdf::colladaToFile(dom, test_collada_string));
}

TEST_F(ColladaUrdfTestFixture, collada_from_urdf_xml_works)
{
    TiXmlDocument urdf_xml;
    ASSERT_TRUE(urdf_xml.Parse(readTestUrdfString().c_str()) > 0);

    boost::shared_ptr<DAE> dom;
    ASSERT_TRUE(collada_urdf::colladaFromUrdfXml(&urdf_xml, dom));
    ASSERT_TRUE(collada_urdf::colladaToFile(dom, test_collada_string));
}

TEST_F(ColladaUrdfTestFixture, collada_from_urdf_model_works)
{
    urdf::Model robot_model;
    TiXmlDocument urdf_xml;
    ASSERT_TRUE(urdf_xml.Parse(readTestUrdfString().c_str()) > 0);
    ASSERT_TRUE(robot_model.initXml(&urdf_xml));

    boost::shared_ptr<DAE> dom;
    ASSERT_TRUE(collada_urdf::colladaFromUrdfModel(robot_model, dom));
    ASSERT_TRUE(collada_urdf::colladaToFile(dom, test_collada_string));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
