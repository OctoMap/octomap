#!/usr/bin/env python

# Increases the version number of package.xml and CMakeLists.txt files in
# subfolders. The first argument specifies the version increase:
# major, minor, or patch (default, e.g. 1.6.2 --> 1.6.3)
#
# Borrows heaviliy from ROS / catkin release tools


import re
import sys
import copy

manifest_match = "<version>(\d+)\.(\d+)\.(\d+)</version>"


if __name__ == '__main__':
  bump = "patch"
  if len(sys.argv) > 1:
    bump = sys.argv[1]
    if bump not in {"major","minor","patch"}:
      print sys.argv[0]+" [major|minor|patch] (default: patch)"
      exit(-1)
  
  
  
  manifests=["octomap/package.xml","octovis/package.xml","dynamicEDT3D/package.xml"]
  cmakelists=["octomap/CMakeLists.txt","octovis/CMakeLists.txt","dynamicEDT3D/CMakeLists.txt"]
  versions = []

  # find versions in package.xml
  for manifest in manifests:
    with open(manifest, 'r') as f:
      package_str = f.read()
    match = re.search(manifest_match, package_str)
    if match is None:
      print "Error: no version tag found in %s" % manifest
      exit(-1)
    else:
      v= match.groups()
      v = [int(x) for x in v]
      versions.append(v)

  # find version in CMakeLists:
  for cmake in cmakelists:
    with open(cmake, 'r') as f:
      cmake_str = f.read()
    v = []
    for m in ["MAJOR","MINOR","PATCH"]:
      searchstr = "_%s_VERSION (\d+)\)" % m
      match = re.search(searchstr, cmake_str)
      if match is None:
        print "Error: no version tag %s found in %s" % (searchstr,cmake)
        exit(-1)
      
      v.append(int(match.group(1)))

    versions.append(v)
  
  new_version = copy.deepcopy(versions[0])
  for v in versions:
    if v != versions[0]:
      print "Error: check current versions, mismatch: %d.%d.%d vs. %d.%d.%d" %(tuple(v)+tuple(versions[0]))
      exit(-1)

  print "OctoMap component versions found: %d.%d.%d" % tuple(versions[0])
  # "bump version" from catkin:
  # find the desired index
  idx = dict(major=0, minor=1, patch=2)[bump]
  # increment the desired part
  new_version[idx] += 1
  # reset all parts behind the bumped part
  new_version = new_version[:idx + 1] + [0 for x in new_version[idx + 1:]]
  new_version_str = "%d.%d.%d" % tuple(new_version)
  print 'Updating to new version: %s\n' % new_version_str

  # adjust CMakeLists
  for cmake in cmakelists:
    with open(cmake, 'r') as f:
      cmake_str = f.read()
    idx = dict(MAJOR=0, MINOR=1, PATCH=2)
    for m in ["MAJOR","MINOR","PATCH"]:      
      old_str = "_%s_VERSION %d)" % (m,versions[0][idx[m]])
      new_str = "_%s_VERSION %d)" % (m,new_version[idx[m]])
      cmake_str = cmake_str.replace(old_str, new_str)

    with open(cmake, 'w') as f:
      f.write(cmake_str)

    

  # adjust package.xml
  for manifest in manifests:
    with open(manifest, 'r') as f:
      package_str = f.read()
    old_str = "<version>%d.%d.%d</version>" % tuple(versions[0])
    new_str = "<version>%s</version>" % new_version_str
    new_package_str = package_str.replace(old_str, new_str)

    with open(manifest, 'w') as f:
      f.write(new_package_str)

  print "Finished writing package.xml and CMakeLists.txt files.\n"
  print "Now check the output, adjust CHANGELOG, and \"git commit\".\nFinally, run:"
  print "  git checkout master && git merge --no-ff devel && git tag v%s" % new_version_str
  print "  git push origin master devel && git push --tags"
  print "\n(adjust when not on the \"devel\" branch)\n"

      
  
  
  
  