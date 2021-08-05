### Script to install Boost
BOOST_FOLDER=boost_${BOOST_VERSION//./_}

# Download Boost
wget https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/${BOOST_FOLDER}.tar.gz

# Unzip
tar -zxf ${BOOST_FOLDER}.tar.gz

# Bootstrap
cd ${BOOST_FOLDER}/
./bootstrap.sh --prefix=$CURRDIR/boost_install --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex

# Build and install
sudo ./b2 -j$(sysctl -n hw.logicalcpu) install

# Rebuild ld cache
sudo ldconfig
