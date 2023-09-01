'use strict';

const web = require('..');
const assert = require('assert').strict;

assert.strictEqual(web(), 'Hello from web');
console.info('web tests passed');
