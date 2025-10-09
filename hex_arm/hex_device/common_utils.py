#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
import re
import time
import asyncio
import logging


async def delay(start_time, ms):
    end_time = start_time + ms / 1000
    now = time.perf_counter()
    await asyncio.sleep(end_time - now)


# Create a logger for the hex_device package
_logger = logging.getLogger(__name__.split('.')[0])  # Use 'hex_device' as logger name

def log_warn(message):
    """Log warning message"""
    _logger.warning(message)

def log_err(message):
    """Log error message"""
    _logger.error(message)

def log_info(message):
    """Log info message"""
    _logger.info(message)

def log_common(message):
    """Log common message (info level)"""
    _logger.info(message)

def log_debug(message):
    """Log debug message"""
    _logger.debug(message)
