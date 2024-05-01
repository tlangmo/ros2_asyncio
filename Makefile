SHELL := /bin/bash

lint: ## Format all files
	pre-commit run --all-files

test: ## Run all unit tests
	python3 -m pytest
#################################################################################
# Self Documenting Commands                                                     #
#################################################################################
# Inspired by <http://marmelab.com/blog/2016/02/29/auto-documented-makefile.html>

.DEFAULT_GOAL := help

.PHONY: help lint test

help:  ## Show this help.
	@recipe_max_length=`grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	awk '{print length($$1)}' | sort -nr | head -1`; \
	grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	sort | awk -v recipe_max_length=$$recipe_max_length 'BEGIN {FS = ":.*?## "}; \
	{printf "\033[36m%-" recipe_max_length "s\033[0m %s\n", $$1, $$2}'
