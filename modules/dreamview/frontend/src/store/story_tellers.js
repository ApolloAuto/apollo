import { observable, action } from "mobx";
import _ from 'lodash';

export default class StoryTellers {
    @observable stories = observable.map();

    @action update(world) {
        this.stories.clear();

        if (world.stories) {
            Object.entries(world.stories).forEach(([story, isOn]) => {
                this.stories.set(story, isOn);
            });
        }
    }
}
