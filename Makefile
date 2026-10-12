.PHONY: build run demo clean test
build:
	python3 -m py_compile src/*.py
run:
	python3 src/main.py
demo:
	python3 src/main.py --demo
test:
	python3 -m unittest discover -s tests -v
clean:
	rm -rf src/__pycache__ tests/__pycache__
