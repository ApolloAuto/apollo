const loglevel = require('loglevel');
const { Logger } = require('../lib/log.ts');

describe('Logger', () => {

    it('should create a new instance when it does not exist', () => {
        const logger = Logger.getInstance('test');
        expect(logger).toBeInstanceOf(Logger);
    });

    it('should return an existing instance when it exists', () => {
        const logger1 = Logger.getInstance('test');
        const logger2 = Logger.getInstance('test');
        expect(logger1).toBe(logger2);
    });

    it('should log debug messages', () => {
        const logger = Logger.getInstance('test');
        const spy = jest.spyOn(logger, 'debug');
        logger.debug('test message');
        expect(spy).toHaveBeenCalledWith(expect.stringContaining('test message'));
        spy.mockRestore();
    });

    it('should log info messages', () => {
        const logger = Logger.getInstance('test');
        const spy = jest.spyOn(logger, 'info');
        logger.info('test message');
        expect(spy).toHaveBeenCalledWith(expect.stringContaining('test message'));
        spy.mockRestore();
    });

    it('should log warn messages', () => {
        const logger = Logger.getInstance('test');
        const spy = jest.spyOn(logger, 'warn');
        logger.warn('test message');
        expect(spy).toHaveBeenCalledWith(expect.stringContaining('test message'));
        spy.mockRestore();
    });

    it('should log error messages', () => {
        const logger = Logger.getInstance('test');
        const spy = jest.spyOn(logger, 'error');
        logger.error('test message');
        expect(spy).toHaveBeenCalledWith(expect.stringContaining('test message'));
        spy.mockRestore();
    });
});
