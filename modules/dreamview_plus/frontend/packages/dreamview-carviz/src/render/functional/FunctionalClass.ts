export abstract class FunctionalClass {
    // 激活状态
    public activeState = false;

    public abstract active(): void;

    public abstract deactive(): void;

    public abstract reset?(): void;
}
