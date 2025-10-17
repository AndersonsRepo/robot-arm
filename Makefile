.PHONY: install dev test lint format clean help

help:  ## Show this help message
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

install:  ## Install telearm package in development mode
	pip install -e .

dev:  ## Install development dependencies and setup pre-commit
	pip install -e ".[dev,hardware]"
	pre-commit install

test:  ## Run tests with coverage
	pytest tests/ -v --cov=telearm --cov-report=term-missing

test-fast:  ## Run tests without coverage
	pytest tests/ -v

lint:  ## Run linting checks
	ruff check telearm/ tests/
	black --check telearm/ tests/

format:  ## Format code with black and ruff
	black telearm/ tests/
	ruff check --fix telearm/ tests/

clean:  ## Clean build artifacts and cache files
	rm -rf build/ dist/ *.egg-info .pytest_cache/ .coverage htmlcov/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

build:  ## Build package for distribution
	python -m build

check:  ## Run all quality checks (lint, test)
	$(MAKE) lint
	$(MAKE) test

ci:  ## Run CI pipeline locally
	$(MAKE) clean
	$(MAKE) dev
	$(MAKE) check
	$(MAKE) build

docs:  ## Generate documentation (placeholder)
	@echo "Documentation generation not implemented yet"

examples:  ## Run example scripts
	python -m examples.smoke_test

cli-help:  ## Show CLI help
	telearm --help

cli-test:  ## Test CLI commands (simulation mode)
	telearm --sim home
	telearm --sim status
	telearm --sim calibrate
