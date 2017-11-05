set -ex

main() {
    if [ $TRAVIS_OS_NAME = osx ]; then
        brew update
        brew install freetype
    fi
}

main