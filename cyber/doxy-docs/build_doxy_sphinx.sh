rm -rf xml/*
doxygen Doxyfile
rm -rf build/*
sphinx-build -b html -d build/doctrees source build/html
