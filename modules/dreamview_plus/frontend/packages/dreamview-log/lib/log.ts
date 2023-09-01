import loglevel, { LogLevelNumbers } from 'loglevel';
import { filter, map, of, reduce } from 'rxjs';
import { visualizeFlatted } from './utils/visualizeFlatted';

export enum LogLevel {
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
}

type LogMessage = string | object;

export default class Logger {
    // 用于存储Logger实例的Map
    // eslint-disable-next-line no-use-before-define
    private static instances: Map<string, Logger>;

    // loglevel.Logger实例
    private logger: loglevel.Logger & { loggerName?: string };

    // 用于存储实例的名称
    private readonly loggerName: string;

    // 添加属性来存储HTML元素
    private logElement: HTMLElement | null = null;

    // 日志缓冲区
    private logBuffer: HTMLElement[] = [];

    // 处理缓冲区状态
    private isProcessing = false;

    // 构造函数，初始化Logger实例
    private constructor(name: string) {
        this.logger = loglevel.getLogger(name);
        this.loggerName = name ?? 'default';
        this.setLevel('debug'); // 设置默认级别为debug
    }

    // 获取Logger实例的静态方法
    public static getInstance(name: string): Logger {
        if (!this.instances) {
            this.instances = new Map();
        }
        if (!this.instances.has(name)) {
            this.instances.set(name, new Logger(name));
        }
        return this.instances.get(name) as Logger;
    }

    // 设置Logger实例的日志级别
    setLevel(level: loglevel.LogLevelDesc): void {
        this.logger.setLevel(level);
    }

    // 获取Logger实例的日志级别
    getLevel(): LogLevelNumbers {
        return this.logger.getLevel();
    }

    // 获取Logger实例的名称
    getName(): string {
        return this.loggerName;
    }

    // 添加方法来设置HTML元素
    setLogElement(element: HTMLElement): void {
        this.logElement = element;
    }

    // 修改以将日志输出到HTML元素
    private logToElement(level: string, message: string): void {
        if (this.logElement) {
            let color: string;

            switch (level) {
                case 'DEBUG':
                    color = 'blue';
                    break;
                case 'INFO':
                    color = 'green';
                    break;
                case 'WARN':
                    color = 'orange';
                    break;
                case 'ERROR':
                    color = 'red';
                    break;
                default:
                    color = 'black';
            }

            const formattedMessage = this.formatMessage(level, message);
            const logLevelElement = `<span style="color: ${color}; font-weight: bold;">[${level}]</span>`;
            const logMessageElement = `<span style="font-style: italic; color: black;">${formattedMessage}</span>`;

            // 创建一个 div 元素并设置内容
            const logDiv = document.createElement('div');
            logDiv.innerHTML = `${logLevelElement} ${logMessageElement}`;

            // 添加到缓冲区
            this.logBuffer.unshift(logDiv);

            // 如果没有处理中的任务，开始处理缓冲区
            if (!this.isProcessing) {
                this.processLogBuffer();
            }

            // 检查日志数量，如果超过100条，移除最旧的一条
            while (this.logElement.children.length > 500) {
                this.logElement.removeChild(this.logElement.lastChild);
            }
        }
    }

    private processLogBuffer(): void {
        if (this.logBuffer.length === 0) {
            this.isProcessing = false;
            return;
        }

        this.isProcessing = true;

        // 使用 requestAnimationFrame 来批量处理日志
        requestAnimationFrame(() => {
            const fragment = document.createDocumentFragment();

            while (this.logBuffer.length > 0) {
                const logDiv = this.logBuffer.shift();
                fragment.insertBefore(logDiv, fragment.firstChild);
            }

            if (this.logElement.firstChild) {
                this.logElement.insertBefore(fragment, this.logElement.firstChild);
            } else {
                this.logElement.appendChild(fragment);
            }

            this.processLogBuffer();
        });
    }

    // 输出debug级别的日志

    debug(...message: LogMessage[]): void {
        this.logMessage('DEBUG', ...message);
    }

    // 输出info级别的日志

    info(...message: LogMessage[]): void {
        this.logMessage('INFO', ...message);
    }

    // 输出warn级别的日志

    warn(...message: LogMessage[]): void {
        this.logMessage('WARN', ...message);
    }

    // 输出error级别的日志
    error(...message: LogMessage[]): void {
        this.logMessage('ERROR', ...message);
    }

    private logMessage(level: string, ...message: LogMessage[]): void {
        of(...message)
            .pipe(
                filter((msg) => typeof msg === 'string' || (msg && typeof msg === 'object')),
                map((msg) => (typeof msg === 'string' ? msg : visualizeFlatted(msg))),
                reduce((acc, cur) => `${acc} ${cur}`, ''),
            )
            .subscribe((messageMessage) => {
                switch (level) {
                    case 'DEBUG':
                        this.logger.debug(this.formatMessage('DEBUG', messageMessage));
                        break;
                    case 'INFO':
                        this.logger.info(this.formatMessage('INFO', messageMessage));
                        break;
                    case 'WARN':
                        this.logger.warn(this.formatMessage('WARN', messageMessage));
                        break;
                    case 'ERROR':
                        this.logger.error(this.formatMessage('ERROR', messageMessage));
                        break;
                    default:
                        this.logger.info(this.formatMessage('INFO', messageMessage));
                }

                // 检查logElement是否已设置，如果设置，则输出到HTML元素
                if (this.logElement) {
                    this.logToElement(level, messageMessage);
                }
            });
    }

    // 格式化日志信息
    formatMessage(level: string, message: string): string {
        const timestamp = new Date().toISOString();
        // 如果是debug级别的日志，需要在日志信息前面加上当前实例的name属性
        if (this.getLevel() === LogLevel.DEBUG && this.getName() !== 'default') {
            const name = this.getName();
            return `${timestamp} [${name}] [${level}] ${message}`;
        }
        return `${timestamp} [${level}] ${message}`;
    }
}
